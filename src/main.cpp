
#include <sdkconfig.h>
#include <driver/i2c.h>
#include <driver/periph_ctrl.h>
#include <soc/i2c_struct.h>
#include <soc/i2c_reg.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstdint>

#if __has_include(<soc/i2c_periph.h>)
#include <soc/i2c_periph.h>
#endif

namespace i2c_device
{
  static constexpr i2c_port_t I2C_PORT = (i2c_port_t)I2C_NUM_0; // I2C ペリフェラルのポート番号
  static constexpr std::uint8_t I2C_ADDR = 0x11;                // I2C アドレス (有効値は 0x08～0x77の範囲)

  static constexpr gpio_num_t PIN_SDA = (gpio_num_t)21;         // I2C SDAピン番号 
  static constexpr gpio_num_t PIN_SCL = (gpio_num_t)22;         // I2C SCLピン番号

  static constexpr std::uint32_t i2c_rx_fifo_full_thresh_val  =  1; // 受信FIFOバッファがこの数を上回ると割込みが発生する
  static constexpr std::uint32_t i2c_tx_fifo_empty_thresh_val =  2; // 送信FIFOバッファがこの数を下回ると割込みが発生する
  static constexpr std::uint32_t i2c_slave_sda_sample_val     =  4; // SCL立上り後のSDAサンプル時間
  static constexpr std::uint32_t i2c_slave_sda_hold_val       =  4; // SCL立下り後のSDAホールド時間
  static constexpr std::uint32_t i2c_intr_mask = ~0;

  static constexpr std::size_t register_size = 256;       // レジスタの数

  static std::uint8_t register_data[register_size] = {0}; // レジスタデータ

  static std::size_t write_reg_index = 0;  // 書込み先のI2Cレジスタ番号
  static std::size_t read_reg_index = 0;   // 読出し元のI2Cレジスタ番号
  static bool update_reg_index = true;     // レジスタ番号の更新が必要か否かフラグ
  static bool reg_modified = false;        // レジスタデータ更新の有無フラグ


#if __has_include(<soc/i2c_periph.h>)
  static periph_module_t getPeriphModule(i2c_port_t num)
  {
    return i2c_periph_signal[num].module;
  }
  static std::uint8_t getPeriphIntSource(i2c_port_t num)
  {
    return i2c_periph_signal[num].irq;
  }
#else
  static periph_module_t getPeriphModule(i2c_port_t num)
  {
    return num == 0 ? PERIPH_I2C0_MODULE : PERIPH_I2C1_MODULE;
  }
  static std::uint8_t getPeriphIntSource(i2c_port_t num)
  {
    return num == 0 ? ETS_I2C_EXT0_INTR_SOURCE : ETS_I2C_EXT1_INTR_SOURCE;
  }
#endif

#if defined ( CONFIG_IDF_TARGET_ESP32C3 )

  static i2c_dev_t* getDev(i2c_port_t num)
  {
    return &I2C0;
  }
  static std::uint32_t getRxFifoCount(i2c_dev_t* dev)
  {
    return dev->sr.rx_fifo_cnt;
  }
  static std::uint32_t IRAM_ATTR getTxFifoCount(i2c_dev_t* dev)
  {
    return dev->sr.tx_fifo_cnt;
  }
  static void updateDev(i2c_dev_t* dev)
  {
    dev->ctr.conf_upgate = 1;
  }

#else

  static i2c_dev_t* getDev(i2c_port_t num)
  {
    return num == 0 ? &I2C0 : &I2C1;
  }
  static std::uint32_t getRxFifoCount(i2c_dev_t* dev)
  {
    return dev->status_reg.rx_fifo_cnt;
  }
  static std::uint32_t getTxFifoCount(i2c_dev_t* dev)
  {
    return dev->status_reg.tx_fifo_cnt;
  }
  static void updateDev(i2c_dev_t* dev)
  {
  }

#endif                          

  /// I2Cイベントハンドラ
  static void IRAM_ATTR i2c_isr_handler(void *mainTaskHandle)
  {
    auto dev = getDev(I2C_PORT);

    typeof(dev->int_status) int_sts;
    int_sts.val = dev->int_status.val;
    dev->int_clr.val = int_sts.val;

    /// 受信データがあれば取り込む
    auto rx_fifo_cnt = getRxFifoCount(dev);
    if (rx_fifo_cnt)
    {
      /// 受信データ列の最初のデータはレジスタ番号として扱う
      if (update_reg_index)
      {
        update_reg_index = false;
        /// 書込み用と読出し用の両方のレジスタ番号を更新する
        write_reg_index = (dev->fifo_data.val) & 255;
        read_reg_index = write_reg_index;

        /// 送信FIFOバッファをクリアする
        dev->fifo_conf.tx_fifo_rst = 1;
        dev->fifo_conf.tx_fifo_rst = 0;
        --rx_fifo_cnt;
      }

      /// 残りのデータはレジスタデータに反映する
      if (rx_fifo_cnt)
      {
        /// レジスタデータ更新フラグをセット
        reg_modified = true;
        do
        {
          std::uint32_t data = dev->fifo_data.val;
          if (write_reg_index < register_size)
          {
            register_data[write_reg_index] = data;
            ++write_reg_index;
          }
        } while (--rx_fifo_cnt);
      }
    }

    /// 送信FIFOバッファにデータを貯めておく
    while (getTxFifoCount(dev) < 8)
    {
      dev->fifo_data.val = (read_reg_index < register_size)
                         ? register_data[read_reg_index]
                         : 0;
      ++read_reg_index;
    }

    /// 通信が終了している場合
    if (int_sts.trans_complete || int_sts.arbitration_lost)
    {
      /// 次回受信するデータをレジスタ番号として扱うフラグをセット
      update_reg_index = true;

      /// レジスタデータ更新があればメインタスクに通知を行う
      if ( reg_modified )
      {
        reg_modified = false;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(mainTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR();
      }
    }
  }

  /// I2Cスレーブ動作開始処理
  void setupTask(void* mainTaskHandle)
  {
    if ((ESP_OK == i2c_set_pin(I2C_PORT, PIN_SDA, PIN_SCL, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE, I2C_MODE_SLAVE))
    && (ESP_OK == esp_intr_alloc( getPeriphIntSource(I2C_PORT)
                        , ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3
                        , i2c_isr_handler
                        , mainTaskHandle
                        , nullptr
                        )))
    {
      auto dev = getDev(I2C_PORT);
      dev->int_ena.val = 0;

      periph_module_enable(getPeriphModule(I2C_PORT));

      dev->fifo_conf.tx_fifo_rst = 1;
      dev->fifo_conf.tx_fifo_rst = 0;
      dev->fifo_conf.rx_fifo_rst = 1;
      dev->fifo_conf.rx_fifo_rst = 0;

      typeof(dev->ctr) ctrl_reg;
      ctrl_reg.val = 0;
      ctrl_reg.sda_force_out = 1;
      ctrl_reg.scl_force_out = 1;
      dev->ctr.val = ctrl_reg.val;

      dev->slave_addr.addr = I2C_ADDR;
      dev->slave_addr.en_10bit = 0;

      dev->sda_hold.time = i2c_slave_sda_hold_val;
      dev->sda_sample.time = i2c_slave_sda_sample_val;

#if defined ( CONFIG_IDF_TARGET_ESP32C3 )

      dev->ctr.slv_tx_auto_start_en = 1;

      dev->timeout.time_out_value = 31;
      dev->timeout.time_out_en = 0;

      dev->filter_cfg.val = 0;
      dev->filter_cfg.scl_en = 1;
      dev->filter_cfg.scl_thres = 0;
      dev->filter_cfg.sda_en = 1;
      dev->filter_cfg.sda_thres = 0;

#else

      dev->timeout.tout = 0xFFFFF;

      dev->scl_filter_cfg.en = 1;
      dev->scl_filter_cfg.thres = 0;
      dev->sda_filter_cfg.en = 1;
      dev->sda_filter_cfg.thres = 0;

#endif

#if defined ( CONFIG_IDF_TARGET_ESP32C3 ) || defined ( CONFIG_IDF_TARGET_ESP32S2 )

      typeof(dev->fifo_conf) fifo_conf;
      fifo_conf.val = 0;
      fifo_conf.rx_fifo_wm_thrhd = i2c_rx_fifo_full_thresh_val;
      fifo_conf.tx_fifo_wm_thrhd = i2c_tx_fifo_empty_thresh_val;
      dev->fifo_conf.val = fifo_conf.val;

      dev->int_ena.val = I2C_TRANS_COMPLETE_INT_ENA
                       | I2C_ARBITRATION_LOST_INT_ENA
                       | I2C_BYTE_TRANS_DONE_INT_ENA
                       | I2C_TXFIFO_WM_INT_ENA
                       | I2C_RXFIFO_WM_INT_ENA
                       ;

#else

      typeof(dev->fifo_conf) fifo_conf;
      fifo_conf.val = 0;
      fifo_conf.rx_fifo_full_thrhd = i2c_rx_fifo_full_thresh_val;
      fifo_conf.tx_fifo_empty_thrhd = i2c_tx_fifo_empty_thresh_val;
      dev->fifo_conf.val = fifo_conf.val;

      dev->int_ena.val = I2C_TRANS_COMPLETE_INT_ENA
                       | I2C_ARBITRATION_LOST_INT_ENA
                       | I2C_SLAVE_TRAN_COMP_INT_ENA
                       | I2C_TXFIFO_EMPTY_INT_ENA
                       | I2C_RXFIFO_FULL_INT_ENA
                       ;

#endif

      updateDev(dev);
    }
    vTaskDelete(NULL);
  }
}

void setup(void)
{
  /// I2Cペリフェラルのセットアップをcore0で行う (ISRイベントハンドラをcore0に登録させるため)
  /// core1でセットアップを実行するとISRがcore1で実行されるようになる
  xTaskCreatePinnedToCore(i2c_device::setupTask, "setupTask", 2048, xTaskGetCurrentTaskHandle(), 0, NULL, 0);

  /// WDT (WatchDogTimer)を無効にする
  TaskHandle_t idle = xTaskGetIdleTaskHandleForCPU(0);
  if (idle != nullptr) esp_task_wdt_delete(idle);
#if defined ( APP_CPU_NUM )
  idle = xTaskGetIdleTaskHandleForCPU(APP_CPU_NUM);
  if (idle != nullptr) esp_task_wdt_delete(idle);
#endif
}

void loop(void)
{
  /// レジスタ変更通知があるまでここで待機する
  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

/* ここにレジスタ変更後の処理を書けます
  static int count1 = 0;
  static int count2 = 0;
  if (++count1 == 1024)
  {
    count1 = 0;
    ESP_EARLY_LOGE("loop", "count:%d", ++count2);
  }
*/

}

#if !defined ( ARDUINO )

extern "C" {
  void loopTask(void*)
  {
    setup();
    for (;;) {
      loop();
    }
  }

  void app_main()
  {
#if defined ( APP_CPU_NUM )
    int core = APP_CPU_NUM;
#else
    int core = 0;
#endif

    xTaskCreatePinnedToCore( loopTask
                           , "loopTask"
                           , 8192
                           , NULL
                           , 1
                           , NULL
                           , core
                           );
    vTaskDelete(NULL);
  }
}
#endif
