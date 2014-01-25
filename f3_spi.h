#ifndef F3_SPI_DRIVER_H_
#define F3_SPI_DRIVER_H_
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>

//СТРУКТУРА ФЛАГОВ, СИГНАЛИЗИРУЮЩИХ ОБ ОШИБКЕ В ДРАЙВЕРЕ АЦП
union error_adc {
	int error;
	struct {
		unsigned is_fragmentation :1; //сигнализирует о непрочитанном до конца буфере (обрыв данных)
		unsigned is_passed_sync :1;	    //пропущенная синхронизация
		unsigned is_error_cc_edma :1;//ошибка контроллера каналов EDMA (данный флаг проверяется, если АЦП остановлен adc_status.mode = STOP_MODE)
		unsigned is_error_tc_edma :1;//ошибка контроллера передачи EDMA	(данный флаг проверяется, если АЦП остановлен adc_status.mode = STOP_MODE)
	};
};

//ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ
#define DEVICE_NAME__ "f3_spi"	//имя узла АЦП устройства
#define DEV_CLASS_ADC "MAD - adc"	//имя класса, к которому принадлежат устройства акустического модуля
#define SIZE_FIFO_READ 4 //размер аппаратного FIFO буфера чтения (обязательно должен быть кратен 4 и не превышать 64)
#define BUF_SIZE 480000	//размер DMA буфера (ACNT = 2, BCNT = 2, CCNT = 60 000      x 2

//РЕЖИМЫ РАБОТЫ АЦП
enum adc_mode {
	STOP_MODE, RUN_MODE,
};

//СТРУКТУРА БЛОКА ДАННЫХ, ПЕРЕДАВАЕМОГО ДЛЯ ОБРАБОТКИ В АЦП
struct dataUnit_ADC
{
	union error_adc error;		//ошибки в драйвере АЦП
	enum adc_mode mode;     //режим работы АЦП
	unsigned int amountCount;	//количество отсчётов в блоке данных (1 отс = 4 x 4 байт)
	unsigned int count;			//порядковый номер первого отсчёта в блоке данных
	short* pUnit;					//указатель на блок данных
};

//КОМАНДЫ IOCTL
#define ID_IO_F3_SPI 200	//идентификатор для команд ioctl
//запуск АЦП
#define IOCTL_ADC_START _IO(ID_IO_F3_SPI, 0)
//синхронизация для АЦП
#define IOCTL_ADC_SYNC _IO(ID_IO_F3_SPI, 1)
//остановка АЦП
#define IOCTL_ADC_STOP _IO(ID_IO_F3_SPI, 2)
//приём данных от АЦП
#define IOCTL_ADC_GET_MESSAGE _IOWR(ID_IO_F3_SPI, 0, struct dataUnit_ADC*)
//получение информации о регистрах модулей SPI0 и SPI1 в пространстве ядра
#define IOCTL_READ_REGISTERS _IO(ID_IO_F3_SPI, 5)

#endif /* F3_SPI_DRIVER_H_ */
