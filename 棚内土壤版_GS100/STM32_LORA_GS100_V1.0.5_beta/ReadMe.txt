
项目：		《LoRa无线气象传感器》

程序名称、版本：	STM32_LoRa_SensorV2.0.9

日期：		2018-12-04


该版本主要改动有：

	1.	旧版本将LoRa参数保存在STM32的备份寄存器里，但存在电池掉电后数据丢失的隐患，所以该版本将LoRa参数以两份的数量保存在了EEPROM里。

	2.	新增了开机自检EEPROM机制。每次设备唤醒后自检SN码，LoRa参数等是否校验通过，如果有任何一份数据没有通过，用保存完好的数据将其覆盖恢复。
		即，每次唤醒重启都保证及时的发现储存错误，并总是力求每一份EEPROM里的数据都保存完好。


注意：当新版本程序存在时，老版本程序基本上不再适用于设备上，只适用于代码回顾和代码回滚。
      除非最新版本ReadMe强调说明了只是实验版本，这样就以上一个版本为准。不然以最新版本程序为准。