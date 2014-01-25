/*
 * f3_spi.c
 *
 *  Created on: 04 янв. 2014 г.
 *      Author: andrej
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/omap-dma.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/poll.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_data/edma.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include "f3_spi.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrej Korobchenko <korrav@yandex.ru>");
MODULE_VERSION("0:1.0");

//подсистема прерывания
#define INTCP						0x48200000 //базовый адрес области регистров подсистемы питания
#define INTC_REVISION 				0
#define INTC_SYSCONFIG 				0x10
#define INTC_SYSSTATUS 				0x14
#define INTC_SIR_IRQ  				0x40
#define INTC_CONTROL   				0x48
#define INTC_PROTECTION    			0x4C
#define INTC_IDLE 					0x50
#define INTC_IRQ_PRIORITY  			0x60
#define INTC_THRESHOLD   			0x68
#define INTC_ITR2    				0xC0
#define INTC_MIR2     				0xC4
#define INTC_MIR_CLEAR2      		0xC8
#define INTC_MIR_SET2 				0xCC
#define INTC_ISR_SET2  				0xD0
#define INTC_ISR_CLEAR2   			0xD4
#define INTC_PENDING_IRQ2    		0xD8
#define INTC_ITR3    				0xE0
#define INTC_MIR3     				0xE4
#define INTC_MIR_CLEAR3      		0xE8
#define INTC_MIR_SET3 				0xEC
#define INTC_ISR_SET3  				0xF0
#define INTC_ISR_CLEAR3   			0xF4
#define INTC_PENDING_IRQ3    		0xF8

//подсистема питания
#define CM_PER						0x44E00000 //базовый адрес области регистров подсистемы питания
#define CM_PER_SPI0_CLKCTRL 		0x4C
#define CM_PER_SPI1_CLKCTRL         0x50
#define CM_PER_L4LS_CLKSTCTRL 		0
//регистры SPI
#define OMAP2_MCSPI_REVISION		0x00
#define OMAP2_SYSCONFIG				0x110
#define OMAP2_MCSPI_SYSSTATUS		0x114
#define OMAP2_MCSPI_IRQSTATUS		0x118
#define OMAP2_MCSPI_IRQENABLE		0x11c
#define OMAP2_MCSPI_WAKEUPENABLE	0x120
#define OMAP2_MCSPI_SYST			0x124
#define OMAP2_MCSPI_MODULCTRL		0x128
#define OMAP2__CH0CONF				0x12c
#define OMAP2_CH0STAT				0x130
#define OMAP2_CH0CTRL				0x134
#define OMAP2_TX0					0x138
#define OMAP2_RX0					0x13c
#define OMAP2_MCSPI_XFERLEVEL		0x17c

//bitfield OMAP2_MCSPI_MODULCTRL
#define OMAP2_MCSPI_MODULCTRL_SINGLE	BIT(0)
#define OMAP2_MCSPI_MODULCTRL_PIN34	BIT(1)
#define OMAP2_MCSPI_MODULCTRL_MS	BIT(2)
#define OMAP2_MCSPI_MODULCTRL_STEST	BIT(3)

//bitfield OMAP2__CH0CONF
#define OMAP2_MCSPI_CHCONF_PHA			BIT(0)
#define OMAP2_MCSPI_CHCONF_POL			BIT(1)
#define OMAP2_MCSPI_CHCONF_CLKD_MASK	(0x0f << 2)
#define OMAP2_MCSPI_CHCONF_EPOL			BIT(6)
#define OMAP2_MCSPI_CHCONF_WL_MASK		(0x1f << 7)
#define OMAP2_MCSPI_CHCONF_TRM_RX_ONLY	BIT(12)
#define OMAP2_MCSPI_CHCONF_TRM_TX_ONLY	BIT(13)
#define OMAP2_MCSPI_CHCONF_TRM_MASK		(0x03 << 12)
#define OMAP2_MCSPI_CHCONF_DMAW			BIT(14)
#define OMAP2_MCSPI_CHCONF_DMAR			BIT(15)
#define OMAP2_MCSPI_CHCONF_DPE0			BIT(16)
#define OMAP2_MCSPI_CHCONF_DPE1			BIT(17)
#define OMAP2_MCSPI_CHCONF_IS			BIT(18)
#define OMAP2_MCSPI_CHCONF_TURBO		BIT(19)
#define OMAP2_MCSPI_CHCONF_FORCE		BIT(20)
#define OMAP2_MCSPI_CHCONF_SPIENSLV0	BIT(21)
#define OMAP2_MCSPI_CHCONF_SPIENSLV1	BIT(22)
#define OMAP2_MCSPI_CHCONF_FFER			BIT(28)

#define OMAP2_MCSPI_IRQSTATUS_RX0_FULL  BIT(2)

#define OMAP2_SYSCONFIG_SOFTRESET 		BIT(1)

//bitfield OMAP2_CH0CTRL
#define OMAP2_MCSPI_CHCTRL_EN			BIT(0)

//bitfield OMAP2_MCSPI_XFERLEVEL
#define OMAP2_MCSPI_XFERLEVEL_AFL_MASK  (0x3F << 8)

#define OMAP2_MCSPI_WAKEUPENABLE_WKEN	BIT(0)

#define OMAP2_MCSPI_RX0_FULL__ENABLE	BIT(2)

#define SPI_IRQ 12	//номер прерывания для SPI EDMA

#define NOT_SYNC -1		//в течение заполнения данного блока сигнал синхронизации не получен

#define SAMPL_SIZE 8    //размер одного отсчёта (по всем 4 каналам)

#define COMPL_READ_BUFFER -1	//данный буффер полностью прочитан

//условная переменная, указывающая на то, что произошёл очередной ping-pong
DECLARE_COMPLETION(done_PingPong);
//условная переменная, указывающая на то, что произошёл очередной ping-pong
DECLARE_WAIT_QUEUE_HEAD( qwait);

//переменные для символьного устройства
static char device_name[] = DEVICE_NAME__; //имя устройства
dev_t dev_f3_spi; //содержит старший и младший номер устройства
static struct cdev cdev_f3_spi;
static struct class* devclass; //класс устройств, к которому принадлежит pga2500

enum status_controller {
	UNINITIALIZED = 0, INITIALIZED
};

struct mcspi_dma {
	int dma_rx_sync_dev;
	char dma_rx_ch_name[14];
};

void __iomem *map_cer; //отображение области регистров подсистемы питания
void __iomem *map_inq; //отображение области регистров подсистемы прерывания

struct mcspi_controller { //хранятся аппаратные данные контроллеров spi
	int status; //	характеризует состояние контроллера
	int id; //номер контроллера
	void __iomem *base; //виртуальный базовый адрес регистров контроллера
	unsigned long phys;  //физический базовый адрес регистров контроллера
	struct platform_device *pdev; //указатель на устройство платформы, соотвествующее контроллеру
	struct device_node *pnode; //указатель на узел устройства в DT
	u32 cs;  //номер cs вывода, с которым работает контроллер
	struct mcspi_dma dma; //dma параметры
} control[2];

struct sync_marks {
	unsigned int count_of_first; //значение count для первого отсчёта в буфере
	unsigned int number_of_first; //номер отсчёта в буфере,  указывающий на новую синхронизацию (если нет, то =NOT_SYNC)
};

static struct buffer_spi { //структура состояния буфера
	bool compl_fill; //состояние заполненности буфера (полностью или нет)
	struct sync_marks sync_m; //метки синхронизации для данного буфера
	struct list_head gain; //список моментов изменения коэффициента усиления
	int loc; //текущая позиция, учитываемая при считывании буфера (=COMPL_READ_BUFFER, когда буфер полностью прочитан) 1 ед = 4 x 2 байта;
	char* virt_loc; //виртуальный адрес буфера
	dma_addr_t bus_loc; //шинный адрес буфера
	int size; //размер буфера (в байтах)
	int num_slot; //номер слота буфера
	int num_slot1; //номер слота буфера для spi1
} buf[2];

static struct {
	enum adc_mode mode; //текущий режим работы
	short num_buf_write; //номер заполняемого в данный момент буфера
	short num_buf_read; //номер считываемого в данный момент буфера
	int users; //количество пользователей в данный момент, использующих АЦП
	union error_adc error; //ошибки в драйвере АЦП
	unsigned int gain; //текущий коэффициент усиления
	struct mutex mutex_lock; //мьютекс
	int count; //текущий отсчёт (обновляется при ping-ponge)

} adc_status; //состояние драйвера

struct gain_MAD { //структура, содержащая номер отсчёта и новое значение, которое приобрёл коэффициент усиления, начиная с данного отсчёта
	struct list_head list;
	int gain; //величина коэффициента усиления
	int sampl; //номер отсчёта
};

//функция, сообщающая значения всех регистров модуля spi
void print_registers_spi(void);

//функция для оперирования регистрами контроллеров spi
static inline void mcspi_write_reg(struct mcspi_controller *pcontrol, int idx,
		u32 val) {
	__raw_writel(val, pcontrol->base + idx);
}

static inline u32 mcspi_read_reg(struct mcspi_controller *pcontrol, int idx) {

	return __raw_readl(pcontrol->base + idx);
}

//регистрация в ядре таблицы соответствия для обеспечения механизма сопоставления записи DT и данного драйвера
static const struct of_device_id mcspi_of_match[] = { { .compatible =
		"ti,omap4-mcspi_s0", .data = &control[0], }, { .compatible =
		"ti,omap4-mcspi_s1", .data = &control[1], }, { }, };
MODULE_DEVICE_TABLE(of, mcspi_of_match);

irqreturn_t spi_handler(int req, void* dev);

static void start_adc(void) //запуск АЦП
{
	u32 regval;
	if (adc_status.mode == STOP_MODE) {
		adc_status.error.error = 0;
		regval = 0;
		regval |= OMAP2_MCSPI_CHCTRL_EN;
		mcspi_write_reg(&control[0], OMAP2_CH0CTRL, regval);
		mcspi_write_reg(&control[1], OMAP2_CH0CTRL, regval);
		adc_status.mode = RUN_MODE;
		/*приказ stmf3 начать передачу данных*/
	}
	return;
}

static void stop_adc(void) //остановка АЦП
{
	u32 regval;
	if (adc_status.mode == RUN_MODE) {
		/*сообщение stmf3 об остановке adc*/
		regval = 0;
		regval &= ~OMAP2_MCSPI_CHCTRL_EN;
		mcspi_write_reg(&control[0], OMAP2_CH0CTRL, regval);
		mcspi_write_reg(&control[1], OMAP2_CH0CTRL, regval);
		adc_status.mode = STOP_MODE;
		adc_status.error.is_fragmentation = 1;
		adc_status.error.is_passed_sync = 1;
	}
	return;
}

//ФУНКЦИИ СТРУКТУРЫ FILE_OPERATIONS
static int f3_spi_open(struct inode *inode, struct file *filp) {
	adc_status.users++;
	nonseekable_open(inode, filp); //сообщение ядру, что данное устройство не поддерживает произвольный доступ к данным
	return (0);
}

static int f3_spi_release(struct inode *inode, struct file *filp) {
	if (--adc_status.users <= 0)
		stop_adc();
	return (0);
}

unsigned int f3_spi_poll(struct file *filp, poll_table *wait) {
//	printk(KERN_INFO "Function is called poll\n");
	unsigned int mask = 0;
	mutex_lock(&adc_status.mutex_lock);
	disable_irq(SPI_IRQ);
	poll_wait(filp, &qwait, wait);
	if ((adc_status.mode == STOP_MODE)
			|| (buf[adc_status.num_buf_read].compl_fill == false)
			|| (buf[adc_status.num_buf_read].loc == COMPL_READ_BUFFER))
		mask = 0;
	else {
//		//printk(KERN_INFO "Now you can read data\n");
		mask = POLLIN | POLLRDNORM;
	}
	enable_irq(SPI_IRQ);
	mutex_unlock(&adc_status.mutex_lock);
	return (mask);
}

long f3_spi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	struct dataUnit_ADC temp_Unit;
	int buf_cur_size; //размер текущего буфера в единицах SAMPL_SIZE
	dma_addr_t cur_position; //физический адрес текущей позиции заполняемого DMA буфера
	int num_trans_sampl = 0; // количество отсчётов, которое будет передано на данном запросе
	switch (cmd) {
	case IOCTL_ADC_START: //СТАРТ АЦП
		start_adc();
		printk(KERN_INFO "Was launched by ADC\n");
		break;
	case IOCTL_READ_REGISTERS:
		print_registers_spi();
		break;
	case IOCTL_ADC_SYNC: //ПОЛУЧЕНИЕ СИНХРОСИГНАЛА
		//вычисление номера текущего отсчёта
		disable_irq(SPI_IRQ);
		edma_get_position(control[0].dma.dma_rx_sync_dev, NULL, &cur_position);
		buf[adc_status.num_buf_write].sync_m.number_of_first = (cur_position
				- buf[adc_status.num_buf_write].bus_loc) / SAMPL_SIZE;
		adc_status.error.is_passed_sync = 0;
		//printk(KERN_INFO "numer first sampl = %u\n",
		//		buf[pcm4204_status.num_buf_write].sync_m.number_of_first);
		enable_irq(SPI_IRQ);
		break;
	case IOCTL_ADC_STOP:
		stop_adc();
		printk(KERN_INFO "Was stopped by the ADC\n");
		break;
	case IOCTL_ADC_GET_MESSAGE: //ПОЛУЧЕНИЕ БЛОКА ДАННЫХ АЦП
		//получение буфера пользовательского пространства
		if (copy_from_user(&temp_Unit, (void*) arg,
				sizeof(struct dataUnit_ADC)))
			return (-EFAULT);
		temp_Unit.error.error = 0;
		temp_Unit.mode = adc_status.mode;
		mutex_lock(&adc_status.mutex_lock);
		disable_irq(SPI_IRQ);
		printk(KERN_INFO "request IOCTL_ADC_GET_MESSAGE\n");
//		printk(
//				KERN_INFO "Number of the first frame in the buffer is equal to %d\n",
//				temp_Unit.count);
		//проверка наблюдается ли фрагментация данных
		if (adc_status.error.is_fragmentation == 1) {
			//printk(KERN_INFO "There is fragmentation of data\n");
			temp_Unit.error.is_fragmentation = 1;
			adc_status.error.is_fragmentation = 0;
		}
//			printk(KERN_INFO "There has been no data fragmentation\n");
		//проверка наблюдается ли пропущенная синхронизация данных
		if (adc_status.error.is_passed_sync == 1) {
			//printk(KERN_INFO "There is a missed sync\n");
			temp_Unit.error.is_passed_sync = 1;
		}
		//printk(KERN_INFO "Nor is there a missed sync\n");
		//возможно ли сейчас передать очередной блок данных АЦП?
		if (temp_Unit.mode == STOP_MODE) { //проверка на наличие фатальных ошибок
			printk(KERN_INFO "ADC now stopped\n");
			temp_Unit.error.is_error_cc_edma =
					adc_status.error.is_error_cc_edma;
			temp_Unit.error.is_error_tc_edma =
					adc_status.error.is_error_tc_edma;
			temp_Unit.amountCount = 0;
			mutex_unlock(&adc_status.mutex_lock);
			enable_irq(SPI_IRQ);
			return (0);
		} else if ((buf[adc_status.num_buf_read].compl_fill == false)
				|| (buf[adc_status.num_buf_read].loc == COMPL_READ_BUFFER)) {
			//ожидание, когда произойдёт ротация буферов АЦП (ping-pong)
printk(KERN_INFO "At the moment it is impossible to transmit data\n");
																																	INIT_COMPLETION(done_PingPong);
			enable_irq(SPI_IRQ);
			wait_for_completion(&done_PingPong);
			//printk(KERN_INFO "It is now possible to transfer data\n");
			disable_irq(SPI_IRQ);
		}
		//заполнение поля первого отсчёта в пользовательском буфере
		if (buf[adc_status.num_buf_read].sync_m.number_of_first == NOT_SYNC
				|| buf[adc_status.num_buf_read].loc
						< buf[adc_status.num_buf_read].sync_m.number_of_first)
			temp_Unit.count = buf[adc_status.num_buf_read].sync_m.count_of_first
					+ buf[adc_status.num_buf_read].loc;
		else
			temp_Unit.count = buf[adc_status.num_buf_read].loc
					- buf[adc_status.num_buf_read].sync_m.number_of_first;
		buf_cur_size = buf[adc_status.num_buf_read].size / SAMPL_SIZE;
		//вычисление сколько данных будет передано
		if (temp_Unit.amountCount
				> (buf_cur_size - buf[adc_status.num_buf_read].loc)) { //возможно ли передать полностью запрошенный объём данных?
			num_trans_sampl = buf_cur_size - buf[adc_status.num_buf_read].loc;
		} else
			num_trans_sampl = temp_Unit.amountCount;
		//	printk(KERN_INFO "Will be transferred to %d sampls\n", num_trans_sampl);
		//проверка на наличие отметки синхронизации на текущей транзакции
		if (buf[adc_status.num_buf_read].sync_m.number_of_first != NOT_SYNC
				&& buf[adc_status.num_buf_read].sync_m.number_of_first
						> buf[adc_status.num_buf_read].loc
				&& buf[adc_status.num_buf_read].sync_m.number_of_first
						<= buf[adc_status.num_buf_read].loc + num_trans_sampl) {
			num_trans_sampl =
					buf[adc_status.num_buf_read].sync_m.number_of_first
							- buf[adc_status.num_buf_read].loc;
			//printk(KERN_INFO "At the current transaction is synchronized\n");
		}
		temp_Unit.amountCount = num_trans_sampl;
		//заполнение буфера пользовательского пространства отсчётами АЦП
		if (copy_to_user(temp_Unit.pUnit,
				buf[adc_status.num_buf_read].virt_loc
						+ buf[adc_status.num_buf_read].loc * SAMPL_SIZE,
				num_trans_sampl * SAMPL_SIZE))
			return (-EFAULT);
		//printk(KERN_INFO "In custom yanked %d bytes\n", num_trans_sampl * SAMPL_SIZE);
		//обновление текущей позиции в считываемом буфере
		buf[adc_status.num_buf_read].loc += num_trans_sampl;
		if (buf[adc_status.num_buf_read].loc >= buf_cur_size)//считан ли до конца текущий буфер?
			buf[adc_status.num_buf_read].loc = COMPL_READ_BUFFER;
		//printk(KERN_INFO "Current position in buffer %d was equal to %d readings\n",
		//adc_status.num_buf_read, buf[adc_status.num_buf_read].loc);
		//передача заполненного буфера АЦП процессу пользовательского пространства
		enable_irq(SPI_IRQ);
		mutex_unlock(&adc_status.mutex_lock);
		if (copy_to_user((void*) arg, &temp_Unit, sizeof(struct dataUnit_ADC)))
			return (-EFAULT);
		return (num_trans_sampl);
		break;
	}
	return (0);
}

//СТРУКТУРА FILE_OPERATIONS
static const struct file_operations f3_spi_fops = { .owner = THIS_MODULE,
		.open = f3_spi_open, .release = f3_spi_release, .poll = f3_spi_poll,
		.unlocked_ioctl = f3_spi_ioctl };

//функция обратного вызова для канала spi0 (вызывается после полного заполнения буфера Dma)
static void callback_dma(unsigned lch, u16 ch_status, void *data) {
	int buf_temp = adc_status.num_buf_read;
	switch (ch_status) {
	case DMA_COMPLETE: //нормальное завершение передачи EDMA
		//фиксирование факта, что только что записанный буфер полностью заполнен
		buf[adc_status.num_buf_write].compl_fill = true;
		//установка текущей позиции считывания в начало только что записанного буфера
		buf[adc_status.num_buf_write].loc = 0;
		//успел ли считаться предыдущий буфер ?
		if (buf[adc_status.num_buf_read].loc != COMPL_READ_BUFFER)
			//printk(KERN_INFO "Previous buffer does not have time to fully read\n");
			adc_status.error.is_fragmentation = 1; //предыдущий буфер не был полностью считан
		//обновление счётчика
		if (buf[adc_status.num_buf_read].sync_m.number_of_first == NOT_SYNC)
			adc_status.count += buf[adc_status.num_buf_read].size / SAMPL_SIZE;
		else
			adc_status.count = (buf[adc_status.num_buf_read].size / SAMPL_SIZE)
					- buf[adc_status.num_buf_read].sync_m.number_of_first;
		buf[adc_status.num_buf_write].sync_m.count_of_first = adc_status.count;
		buf[adc_status.num_buf_read].sync_m.number_of_first = NOT_SYNC;
//		printk(KERN_INFO "Now the count is %d\n", pcm4204_status.count);
		//фиксирование факта, что только что считываемый буфер пуст
		buf[adc_status.num_buf_read].compl_fill = false;
		//изменение индексов считываемого и записываемого буфера
		adc_status.num_buf_read = adc_status.num_buf_write;
		adc_status.num_buf_write = buf_temp;
		//printk(KERN_INFO "Only that there was a ping-pong\n");
		complete(&done_PingPong); //сигнализирование о произошедшей ротации буферов (ping-pong)
		wake_up_interruptible(&qwait);
		//сигнализирование о доступности данных для чтения
		break;
	case DMA_CC_ERROR: //ошибка контроллера каналов
		printk(KERN_ALERT "there was an error of the controler of channels\n");
		stop_adc(); //остановка АЦП
		adc_status.error.is_error_cc_edma = 1;
		complete(&done_PingPong); //сигнализирование об ошибке
		break;
	case DMA_TC1_ERROR:
	case DMA_TC2_ERROR:
		printk(
				KERN_ALERT "there was an error of the controler of transmissions\n");
		stop_adc(); //остановка АЦП
		adc_status.error.is_error_tc_edma = 1;
		complete(&done_PingPong); //сигнализирование об ошибке
		break;
	}
}

//функция инициализации spi контроллеров и  структур модуля
static int omap2_mcspi_probe(struct platform_device *pdev) {
	struct edmacc_param param_set_buf[2]; //PARAM для буферов spi0
	u32 regval;
	int result;
	struct mcspi_controller* pcontrol;
	struct resource *r;
	const struct of_device_id *match;
	struct pinctrl *pinctrl;
	unsigned sig;
	int acnt = 2, bcnt = 2, ccnt = 60000, dst_bindex = 2, dst_cindex = 8; //параметры каналов spi
	int status = 0;
	struct device *dev;

	match = of_match_device(mcspi_of_match, &pdev->dev);

	//возвращение адреса структуры mcspi_controller, принадлежащей данному устройству platform_device
	if (match) {
		pcontrol = (struct mcspi_controller*) match->data;
	} else {
		dev_err(&pdev->dev, "no match is found with the match table\n");
		return -EPERM;
	}

	//инициализация структуры mcspi_controller, принадлежащей данному устройству platform_device
	pcontrol->pdev = pdev;
	pcontrol->pnode = pdev->dev.of_node;
	pcontrol->id = (pcontrol == &control[0] ? 0 : 1);
	of_property_read_u32(pcontrol->pnode, "ti,cs", &pcontrol->cs);
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "impossible to obtain the memory resources\n");
		return -EPERM;
	}
	pcontrol->phys = r->start;
	pcontrol->base = devm_request_and_ioremap(&pdev->dev, r);

	if (!pcontrol->base) {
		dev_err(&pdev->dev, "can't ioremap MCSPI\n");
		return -ENOMEM;
	}
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "pins are not configured from the driver\n");
		return -EPERM;
	}
	strcpy(pcontrol->dma.dma_rx_ch_name, "rx0");
	r = platform_get_resource_byname(pdev,
	IORESOURCE_DMA, pcontrol->dma.dma_rx_ch_name);
	if (r == NULL) {
		dev_err(&pdev->dev, "impossible to obtain the dma resources\n");
		return -ENODEV;
	}
	pcontrol->dma.dma_rx_sync_dev = r->start;
	//включение модуля
	pm_runtime_enable(&pdev->dev);
	if (pm_runtime_get_sync(&pdev->dev) < 0) {
		dev_err(&pdev->dev, "impossible power on the module\n");
		return -ENODEV;
	}
	/*mcspi_write_reg(pcontrol, OMAP2_MCSPI_WAKEUPENABLE,
	 OMAP2_MCSPI_WAKEUPENABLE_WKEN);*/
	//инициализация регистров модуля
	mcspi_write_reg(pcontrol, OMAP2_SYSCONFIG, 0x308);

	mcspi_write_reg(pcontrol, OMAP2_MCSPI_MODULCTRL,
	OMAP2_MCSPI_MODULCTRL_MS | OMAP2_MCSPI_MODULCTRL_PIN34);
	regval = 0;
	regval &= ~OMAP2_MCSPI_CHCONF_DPE0;
	regval |= OMAP2_MCSPI_CHCONF_DPE1;
	regval |= OMAP2_MCSPI_CHCONF_IS;
	regval &= ~OMAP2_MCSPI_CHCONF_PHA;
	regval &= ~OMAP2_MCSPI_CHCONF_POL;
	regval |= OMAP2_MCSPI_CHCONF_EPOL;
	regval &= ~OMAP2_MCSPI_CHCONF_WL_MASK;
	regval |= 15 << 7;
	regval &= ~OMAP2_MCSPI_CHCONF_TRM_MASK;
	regval |= OMAP2_MCSPI_CHCONF_TRM_RX_ONLY;
	regval &= ~ OMAP2_MCSPI_CHCONF_DMAW;
	regval |= OMAP2_MCSPI_CHCONF_DMAR;
	regval &= ~OMAP2_MCSPI_CHCONF_TURBO;
	regval &= ~ OMAP2_MCSPI_CHCONF_SPIENSLV0;
	regval &= ~ OMAP2_MCSPI_CHCONF_SPIENSLV1;
	regval |= OMAP2_MCSPI_CHCONF_FFER;
	mcspi_write_reg(pcontrol, OMAP2__CH0CONF, regval);

	regval = 0;
	regval &= ~OMAP2_MCSPI_XFERLEVEL_AFL_MASK;
	regval |= 3 << 8;
	mcspi_write_reg(pcontrol, OMAP2_MCSPI_XFERLEVEL, regval);

	regval = 0;
	regval |= OMAP2_MCSPI_RX0_FULL__ENABLE;
	mcspi_write_reg(pcontrol, OMAP2_MCSPI_IRQENABLE, regval);

	sig = pcontrol->dma.dma_rx_sync_dev;
	pcontrol->status = INITIALIZED;

	if (control[0].status == INITIALIZED && control[1].status == INITIALIZED) {
		//ИНИЦИАЛИЗАЦИЯ EDMA
		//выделение буферов из памяти и инициализация структур состояния буферов драйвера
		buf[0].compl_fill = false;
		buf[0].loc = 0;
		buf[0].size = BUF_SIZE;
		INIT_LIST_HEAD(&buf[0].gain);
		buf[0].sync_m.count_of_first = 0;
		buf[0].sync_m.number_of_first = NOT_SYNC;
		buf[0].virt_loc = dma_zalloc_coherent(NULL, buf[0].size,
				&buf[0].bus_loc,
				GFP_KERNEL | GFP_DMA); //размещение и отображение для буфера buf[0]
		if (!buf[0].virt_loc) {
			printk(KERN_ALERT "dma_alloc_coherent failed for buf[0]\n");
			status = -ENOMEM;
			goto disable_pm;
		}
		buf[1].compl_fill = false;
		buf[1].loc = 0;
		buf[1].size = BUF_SIZE;
		INIT_LIST_HEAD(&buf[1].gain);
		buf[1].sync_m.count_of_first = 0;
		buf[1].sync_m.number_of_first = NOT_SYNC;
		buf[1].virt_loc = dma_alloc_coherent(NULL, buf[1].size, &buf[1].bus_loc,
		GFP_KERNEL | GFP_DMA); //размещение и отображение для буфера buf[1]
		if (!buf[1].virt_loc) {
			printk(KERN_ALERT "dma_alloc_coherent failed for buf[1]\n");
			status = -ENOMEM;
			goto free_memory_buf0;
		}

		//выделение каналов DMA для spi0 и spi1
		result = edma_alloc_channel(control[0].dma.dma_rx_sync_dev,
				callback_dma,
				NULL, EVENTQ_DEFAULT); //размещение DMA канала для spi0
		if (result < 0) {
			printk(KERN_ALERT "edma_alloc_channel for spi0 failed\n");
			status = result;
			goto free_memory_buf1;
		}
		result = edma_alloc_channel(control[1].dma.dma_rx_sync_dev, NULL,
		NULL, EVENTQ_DEFAULT); //размещение DMA канала для spi1
		if (result < 0) {
			printk(KERN_ALERT "edma_alloc_channel for spi1 failed\n");
			status = result;
			goto free_channel_spi0;
		}
		//ИНИЦИАЛИЗАЦИЯ СТРУКТУР PARAMSET для spi0
		edma_set_src(control[0].dma.dma_rx_sync_dev,
				(unsigned long) (control[0].phys + OMAP2_RX0), FIFO, W16BIT); //установка адреса источника и его адресного режима
		printk(KERN_ALERT "SPI0: address source EDMA = %ld\n", control[0].phys + OMAP2_RX0);
		edma_set_dest(control[0].dma.dma_rx_sync_dev,
				(unsigned long) (buf[0].bus_loc), INCR, W16BIT); //установка адреса приёмника и его адресного режима
		edma_set_src_index(control[0].dma.dma_rx_sync_dev, 0, 0); //установка параметров индексации адреса источника
		edma_set_dest_index(control[0].dma.dma_rx_sync_dev, dst_bindex,
				dst_cindex); //установка параметров индексации адреса приёмника
		edma_set_transfer_params(control[0].dma.dma_rx_sync_dev, acnt, bcnt,
				ccnt, bcnt, ABSYNC); //установка счётчиков и режима синхронизации
		//заполнение первого PARAMSET
		edma_read_slot(control[0].dma.dma_rx_sync_dev, &param_set_buf[0]);
		__clear_bit(ITCCHEN, (unsigned long*) &param_set_buf[0].opt); //отключение функции интрацепочной передачи
		__clear_bit(TCCHEN, (unsigned long*) &param_set_buf[0].opt); //отключение функции цепочной передачи
		__clear_bit(ITCINTEN, (unsigned long*) &param_set_buf[0].opt); //запрещение промежуточного прерывания
		__set_bit(TCINTEN, (unsigned long*) &param_set_buf[0].opt); //разрешение прерывания по окончанию передачи
		param_set_buf[0].opt |= EDMA_TCC(
				EDMA_CHAN_SLOT(control[0].dma.dma_rx_sync_dev)); //идентифицирование кода завершения передачи
		__clear_bit(TCCMODE, (unsigned long*) &param_set_buf[0].opt); //установка режима нормального завершения
		__clear_bit(STATIC, (unsigned long*) &param_set_buf[0].opt); //PARAMSET не статичный
		edma_write_slot(control[0].dma.dma_rx_sync_dev, &param_set_buf[0]); //установка PARAMSET для канала
		//заполнение неотображённого слота для buf[0]
		result = edma_alloc_slot(0, EDMA_SLOT_ANY);
		if (result < 0) {
			printk(KERN_ALERT "edma_alloc_slot 0 for spi0 failed for buf[0]\n");
			status = result;
			goto free_channel_spi1;
		}
		buf[0].num_slot = result;
		edma_write_slot(buf[0].num_slot, &param_set_buf[0]);
		//заполнение неотображённого слота для buf[1]
		result = edma_alloc_slot(0, EDMA_SLOT_ANY);
		if (result < 0) {
			printk(KERN_ALERT "edma_alloc_slot 1 for spi0 failed for buf[1]\n");
			status = result;
			goto free_slot0_spi0;
		}
		buf[1].num_slot = result;
		param_set_buf[1] = param_set_buf[0];
		edma_write_slot(buf[1].num_slot, &param_set_buf[1]);
		edma_set_dest(buf[1].num_slot, (unsigned long) (buf[1].bus_loc), INCR,
				W16BIT); //установка адреса приёмника и его адресного режима
		edma_read_slot(buf[1].num_slot, &param_set_buf[1]);
		//связывание канала и слотов между собой - организация пинг-понга
		edma_link(control[0].dma.dma_rx_sync_dev, buf[1].num_slot);
		edma_link(buf[1].num_slot, buf[0].num_slot);
		edma_link(buf[0].num_slot, buf[1].num_slot);

		//ИНИЦИАЛИЗАЦИЯ СТРУКТУР PARAMSET для spi1
		__clear_bit(TCINTEN, (unsigned long*) &param_set_buf[0].opt); //запрещение прерывания по окончанию передачи
		edma_write_slot(control[1].dma.dma_rx_sync_dev, &param_set_buf[0]); //установка PARAMSET для канала
		edma_set_src(control[1].dma.dma_rx_sync_dev,
				(unsigned long) (control[1].phys + OMAP2_RX0), FIFO, W16BIT); //установка адреса источника и его адресного режима
		printk(KERN_ALERT "SPI1: address source EDMA = %ld\n", control[1].phys + OMAP2_RX0);
		edma_set_dest(control[1].dma.dma_rx_sync_dev,
				(unsigned long) (buf[0].bus_loc + 4), INCR, W16BIT); //установка адреса приёмника и его адресного режима
		edma_read_slot(control[1].dma.dma_rx_sync_dev, &param_set_buf[0]);
		//заполнение неотображённого слота для buf[0] для spi1
		result = edma_alloc_slot(0, EDMA_SLOT_ANY);
		if (result < 0) {
			printk(KERN_ALERT "edma_alloc_slot 0 for spi1 failed for buf[0]\n");
			status = result;
			goto free_slot1_spi0;
		}
		buf[0].num_slot1 = result;
		edma_write_slot(buf[0].num_slot1, &param_set_buf[0]);
		//заполнение неотображённого слота для buf[1] для spi1
		result = edma_alloc_slot(0, EDMA_SLOT_ANY);
		if (result < 0) {
			printk(KERN_ALERT "edma_alloc_slot 1 for spi1 failed for buf[1]\n");
			status = result;
			goto free_slot0_spi1;
		}
		buf[1].num_slot1 = result;
		//заполнение неотображённого слота для buf[1] для spi1
		__clear_bit(TCINTEN, (unsigned long*) &param_set_buf[1].opt); //запрещение прерывания по окончанию передачи
		edma_write_slot(buf[1].num_slot1, &param_set_buf[1]);
		edma_set_dest(buf[1].num_slot1, (unsigned long) (buf[1].bus_loc + 4),
				INCR, W16BIT); //установка адреса приёмника и его адресного режима
		//связывание канала и слотов между собой - организация пинг-понга
		edma_link(control[1].dma.dma_rx_sync_dev, buf[1].num_slot1);
		edma_link(buf[1].num_slot1, buf[0].num_slot1);
		edma_link(buf[0].num_slot1, buf[1].num_slot1);
		//запуск EDMA каналов
		result = edma_start(control[0].dma.dma_rx_sync_dev);
		if (result != 0) {
			printk(KERN_ALERT "fail start edma for spi0\n");
			status = result;
			goto free_slot1_spi1;
		}
		result = edma_start(control[1].dma.dma_rx_sync_dev);
		if (result != 0) {
			printk(KERN_ALERT "fail start edma for spi1\n");
			status = result;
			goto stop_edma_spi0;
		}
		//ИНИЦИАЛИЗАЦИЯ ПРЕРЫВАНИЙ
		/*map_inq = ioremap(INTCP, 4096);
		if(!map_inq)
			printk(KERN_INFO"false ioremap(INTCP, 4096)");
		//разрешение прерывания
		__raw_writel(0x2, map_inq + INTC_MIR_CLEAR2);
		__raw_writel(0x20000000, map_inq + INTC_MIR_CLEAR3);
		if (request_irq(65, spi_handler, IRQF_DISABLED | IRQF_NO_SUSPEND, "spi0", &control[0])) {
			printk(KERN_ALERT "fail reuest interrupt for spi0\n");
			status = -EIO;
			goto stop_edma_spi1;
		}
		if (request_irq(125, spi_handler, IRQF_DISABLED | IRQF_NO_SUSPEND, "spi1", &control[1])) {
			printk(KERN_ALERT "fail reuest interrupt for spi1\n");
			status = -EIO;
			goto stop_edma_spi1;
		}*/
		//enable_irq(65);
		//enable_irq(125);

		mcspi_write_reg(&control[0], OMAP2_CH0CTRL, OMAP2_MCSPI_CHCTRL_EN);
		mcspi_write_reg(&control[1], OMAP2_CH0CTRL, OMAP2_MCSPI_CHCTRL_EN);

		map_cer = ioremap(CM_PER, 100);
		if(!map_cer)
		printk(KERN_INFO"false ioremap(CM_PER, 100)");
		//ИНИЦИАЛИЗАЦИЯ ПЕРЕМЕННОЙ СОСТОЯНИЯ ДРАЙВЕРА
		adc_status.mode = STOP_MODE;
		adc_status.num_buf_write = 0;
		adc_status.num_buf_read = 1;
		adc_status.count = -buf[adc_status.num_buf_read].size / SAMPL_SIZE;
		adc_status.error.error = 0;
		adc_status.users = 0;
		adc_status.gain = 1;
		mutex_init(&adc_status.mutex_lock);
		//получение идентификатора для устройства
		result = alloc_chrdev_region(&dev_f3_spi, 0, 1, device_name);
		if (result) {
			printk(KERN_ALERT "fail function alloc_chrdev_region\n");
			status = result;
			goto stop_edma_spi1;
		}
		//регистрация символьного устройства
		cdev_init(&cdev_f3_spi, &f3_spi_fops);
		result = cdev_add(&cdev_f3_spi, dev_f3_spi, 1);
		if (result) {
			printk(KERN_ERR "The cdev_add function failed\n");
			status = result;
			goto alloc_chrdev;
		}
		devclass = class_create(THIS_MODULE, DEV_CLASS_ADC); //создание класса
		if (IS_ERR(devclass)) {
			printk(KERN_ERR "The class_create function failed\n");
			status = (int) PTR_ERR(devclass);
			goto del_cdev;
		}
		dev = device_create(devclass, NULL, dev_f3_spi, NULL, device_name); //создание устройства
		result = IS_ERR(dev) ? PTR_ERR(dev) : 0;
		if (result) {
			printk(KERN_ERR "The device_create function failed\n");
			status = result;
			goto des_class;
		}
	}
	return status;
	des_class: class_destroy(devclass);
	del_cdev: cdev_del(&cdev_f3_spi);
	alloc_chrdev: unregister_chrdev_region(dev_f3_spi, 1);
	stop_edma_spi1: edma_stop(control[1].dma.dma_rx_sync_dev);
	stop_edma_spi0: edma_stop(control[0].dma.dma_rx_sync_dev);
	free_slot1_spi1: edma_free_slot(buf[1].num_slot1);
	free_slot0_spi1: edma_free_slot(buf[0].num_slot1);
	free_slot1_spi0: edma_free_slot(buf[1].num_slot);
	free_slot0_spi0: edma_free_slot(buf[0].num_slot);
	free_channel_spi1: edma_free_channel(control[1].dma.dma_rx_sync_dev);
	free_channel_spi0: edma_free_channel(control[0].dma.dma_rx_sync_dev);
	free_memory_buf1: dma_free_coherent(NULL, buf[1].size, buf[1].virt_loc,
			buf[1].bus_loc);
	free_memory_buf0: dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc,
			buf[0].bus_loc);
	disable_pm: pm_runtime_disable(&control[0].pdev->dev);
	pm_runtime_disable(&control[1].pdev->dev);
	return status;
}

static int omap2_mcspi_remove(struct platform_device *pdev) {
	stop_adc();
	class_destroy(devclass);
	cdev_del(&cdev_f3_spi);
	unregister_chrdev_region(dev_f3_spi, 1);
	edma_stop(control[1].dma.dma_rx_sync_dev);
	edma_stop(control[0].dma.dma_rx_sync_dev);
	edma_free_slot(buf[1].num_slot1);
	edma_free_slot(buf[0].num_slot1);
	edma_free_slot(buf[1].num_slot);
	edma_free_slot(buf[0].num_slot);
	edma_free_channel(control[1].dma.dma_rx_sync_dev);
	edma_free_channel(control[0].dma.dma_rx_sync_dev);
	dma_free_coherent(NULL, buf[1].size, buf[1].virt_loc, buf[1].bus_loc);
	dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc, buf[0].bus_loc);
	pm_runtime_disable(&control[0].pdev->dev);
	pm_runtime_disable(&control[1].pdev->dev);
	return 0;
}
static struct platform_driver omap2_mcspi_driver = { .driver = { .name =
		"f3_spi", .owner = THIS_MODULE, .of_match_table = mcspi_of_match, },
		.probe = omap2_mcspi_probe, .remove = omap2_mcspi_remove, };

module_platform_driver(omap2_mcspi_driver);

irqreturn_t spi_handler(int req, void* dev) {
	u32 val = 0, regval = 0;
	struct mcspi_controller* pcontroller = dev;
	printk(KERN_INFO "Message from interrupt%d\n", req);
	if ((regval = mcspi_read_reg(pcontroller,
	OMAP2_MCSPI_IRQSTATUS)) & OMAP2_MCSPI_IRQSTATUS_RX0_FULL) {
		val = mcspi_read_reg(pcontroller, OMAP2_RX0);
		printk(KERN_INFO "Message %d is received from the SPI %d\n", val,pcontroller->id);
		val |= OMAP2_MCSPI_IRQSTATUS_RX0_FULL;
		mcspi_write_reg(pcontroller, OMAP2_MCSPI_IRQSTATUS, val);
	}
	return IRQ_HANDLED;
}
void print_registers_spi(void) {
	//ВЫВОД ЗНАЧЕНИЙ ВСЕХ РЕГИСТРОВ
printk(KERN_EMERG "Register CM_PER_L4LS_CLKSTCTRL val = %X\n", __raw_readl(map_cer + CM_PER_L4LS_CLKSTCTRL));
printk(KERN_EMERG "Register CM_PER_SPI0_CLKCTRL val = %X\n", __raw_readl(map_cer + CM_PER_SPI0_CLKCTRL));
printk(KERN_EMERG "Register CM_PER_SPI1_CLKCTRL val = %X\n", __raw_readl(map_cer + CM_PER_SPI1_CLKCTRL));
printk(KERN_EMERG "SPI0: register OMAP2_MCSPI_REVISION: address = %ld,  value = %X\n",
		control[0].phys + OMAP2_MCSPI_REVISION,
		mcspi_read_reg(&control[0], OMAP2_MCSPI_REVISION));
printk(KERN_EMERG "SPI1: register OMAP2_MCSPI_REVISION: address = %ld,  value = %X\n",
		control[1].phys + OMAP2_MCSPI_REVISION,
		mcspi_read_reg(&control[1], OMAP2_MCSPI_REVISION));

printk(KERN_EMERG "SPI0: register OMAP2_SYSCONFIG: address = %ld,  value = %X\n",
		control[0].phys + OMAP2_SYSCONFIG,
		mcspi_read_reg(&control[0], OMAP2_SYSCONFIG));
printk(KERN_EMERG "SPI1: register OMAP2_SYSCONFIG: address = %ld,  value = %X\n",
		control[1].phys + OMAP2_SYSCONFIG,
		mcspi_read_reg(&control[1], OMAP2_SYSCONFIG));

printk(KERN_EMERG "SPI0: register OMAP2_MCSPI_SYSSTATUS: address = %ld,  value = %X\n",
		control[0].phys + OMAP2_MCSPI_SYSSTATUS,
		mcspi_read_reg(&control[0], OMAP2_MCSPI_SYSSTATUS));
printk(KERN_EMERG "SPI1: register OMAP2_MCSPI_SYSSTATUS: address = %ld,  value = %X\n",
		control[1].phys + OMAP2_MCSPI_SYSSTATUS,
		mcspi_read_reg(&control[1], OMAP2_MCSPI_SYSSTATUS));

printk(KERN_EMERG "SPI0: register OMAP2_MCSPI_IRQSTATUS: address = %ld,  value = %X\n",
		control[0].phys + OMAP2_MCSPI_IRQSTATUS,
		mcspi_read_reg(&control[0], OMAP2_MCSPI_IRQSTATUS));
printk(KERN_EMERG "SPI1: register OMAP2_MCSPI_IRQSTATUS: address = %ld,  value = %X\n",
		control[1].phys + OMAP2_MCSPI_IRQSTATUS,
		mcspi_read_reg(&control[1], OMAP2_MCSPI_IRQSTATUS));

printk(KERN_EMERG "SPI0: register OMAP2_MCSPI_IRQENABLE: address = %ld,  value = %X\n",
		control[0].phys + OMAP2_MCSPI_IRQENABLE,
		mcspi_read_reg(&control[0], OMAP2_MCSPI_IRQENABLE));
printk(KERN_EMERG "SPI1: register OMAP2_MCSPI_IRQENABLE: address = %ld,  value = %X\n",
		control[1].phys + OMAP2_MCSPI_IRQENABLE,
		mcspi_read_reg(&control[1], OMAP2_MCSPI_IRQENABLE));

printk(KERN_EMERG "SPI0: register OMAP2_MCSPI_WAKEUPENABLE: address = %ld,  value = %X\n",
		control[0].phys + OMAP2_MCSPI_WAKEUPENABLE,
		mcspi_read_reg(&control[0], OMAP2_MCSPI_WAKEUPENABLE));
printk(KERN_EMERG "SPI1: register OMAP2_MCSPI_WAKEUPENABLE: address = %ld,  value = %X\n",
		control[1].phys + OMAP2_MCSPI_WAKEUPENABLE,
		mcspi_read_reg(&control[1], OMAP2_MCSPI_WAKEUPENABLE));

printk(KERN_EMERG "SPI0: register OMAP2_MCSPI_SYST: address = %ld,  value = %X\n",
		control[0].phys + OMAP2_MCSPI_SYST,
		mcspi_read_reg(&control[0], OMAP2_MCSPI_SYST));
printk(KERN_EMERG "SPI1: register OMAP2_MCSPI_SYST: address = %ld,  value = %X\n",
		control[1].phys + OMAP2_MCSPI_SYST,
		mcspi_read_reg(&control[1], OMAP2_MCSPI_SYST));

printk(KERN_EMERG "SPI0: register OMAP2_MCSPI_MODULCTRL: address = %ld,  value = %X\n",
		control[0].phys + OMAP2_MCSPI_MODULCTRL,
		mcspi_read_reg(&control[0], OMAP2_MCSPI_MODULCTRL));
printk(KERN_EMERG "SPI1: register OMAP2_MCSPI_MODULCTRL: address = %ld,  value = %X\n",
		control[1].phys + OMAP2_MCSPI_MODULCTRL,
		mcspi_read_reg(&control[1], OMAP2_MCSPI_MODULCTRL));

printk(KERN_EMERG "SPI0: register OMAP2__CH0CONF: address = %ld,  value = %X\n",
		control[0].phys + OMAP2__CH0CONF,
		mcspi_read_reg(&control[0], OMAP2__CH0CONF));
printk(KERN_EMERG "SPI1: register OMAP2__CH0CONF: address = %ld,  value = %X\n",
		control[1].phys + OMAP2__CH0CONF,
		mcspi_read_reg(&control[1], OMAP2__CH0CONF));

printk(KERN_INFO "SPI0: register OMAP2_CH0STAT: address = %ld,  value = %X\n",
		control[0].phys + OMAP2_CH0STAT,
		mcspi_read_reg(&control[0], OMAP2_CH0STAT));
printk(KERN_EMERG "SPI1: register OMAP2_CH0STAT: address = %ld,  value = %X\n",
		control[1].phys + OMAP2_CH0STAT,
		mcspi_read_reg(&control[1], OMAP2_CH0STAT));

printk(KERN_EMERG "SPI0: register OMAP2_CH0CTRL: address = %ld,  value = %X\n",
		control[0].phys + OMAP2_CH0CTRL,
		mcspi_read_reg(&control[0], OMAP2_CH0CTRL));
printk(KERN_EMERG "SPI1: register OMAP2_CH0CTRL: address = %ld,  value = %X\n",
		control[1].phys + OMAP2_CH0CTRL,
		mcspi_read_reg(&control[1], OMAP2_CH0CTRL));

printk(KERN_EMERG "SPI0: register OMAP2_MCSPI_XFERLEVEL: address = %ld,  value = %X\n",
		control[0].phys + OMAP2_MCSPI_XFERLEVEL,
		mcspi_read_reg(&control[0], OMAP2_MCSPI_XFERLEVEL));
printk(KERN_EMERG "SPI1: register OMAP2_MCSPI_XFERLEVEL: address = %ld,  value = %X\n",
		control[1].phys + OMAP2_MCSPI_XFERLEVEL,
		mcspi_read_reg(&control[1], OMAP2_MCSPI_XFERLEVEL));
printk(KERN_EMERG "Register INTC_REVISION val = %X\n", __raw_readl(map_inq + INTC_REVISION));
printk(KERN_EMERG "Register INTC_SYSSTATUS val = %X\n", __raw_readl(map_inq + INTC_SYSSTATUS));
printk(KERN_EMERG "Register INTC_SIR_IRQ val = %X\n", __raw_readl(map_inq + INTC_SIR_IRQ));
printk(KERN_EMERG "Register INTC_CONTROL val = %X\n", __raw_readl(map_inq + INTC_CONTROL));
printk(KERN_EMERG "Register INTC_PROTECTION val = %X\n", __raw_readl(map_inq + INTC_PROTECTION));
printk(KERN_EMERG "Register INTC_IDLE val = %X\n", __raw_readl(map_inq + INTC_IDLE));
printk(KERN_EMERG "Register INTC_IRQ_PRIORITY val = %X\n", __raw_readl(map_inq + INTC_IRQ_PRIORITY));
printk(KERN_EMERG "Register INTC_THRESHOLD val = %X\n", __raw_readl(map_inq + INTC_THRESHOLD));
printk(KERN_EMERG "Register INTC_ITR2 val = %X\n", __raw_readl(map_inq + INTC_ITR2));
printk(KERN_EMERG "Register INTC_MIR2 val = %X\n", __raw_readl(map_inq + INTC_MIR2));
printk(KERN_EMERG "Register INTC_MIR_CLEAR2 val = %X\n", __raw_readl(map_inq + INTC_MIR_CLEAR2));
printk(KERN_EMERG "Register INTC_MIR_SET2 val = %X\n", __raw_readl(map_inq + INTC_MIR_SET2));
printk(KERN_EMERG "Register INTC_ISR_SET2 val = %X\n", __raw_readl(map_inq + INTC_ISR_SET2));
printk(KERN_EMERG "Register INTC_ISR_CLEAR2 val = %X\n", __raw_readl(map_inq + INTC_ISR_CLEAR2));
printk(KERN_EMERG "Register INTC_PENDING_IRQ2 val = %X\n", __raw_readl(map_inq + INTC_PENDING_IRQ2));
printk(KERN_EMERG "Register INTC_ITR3 val = %X\n", __raw_readl(map_inq + INTC_ITR3));
printk(KERN_EMERG "Register INTC_MIR3 val = %X\n", __raw_readl(map_inq + INTC_MIR3));
printk(KERN_EMERG "Register INTC_MIR_CLEAR3 val = %X\n", __raw_readl(map_inq + INTC_MIR_CLEAR3));
printk(KERN_EMERG "Register INTC_MIR_SET3 val = %X\n", __raw_readl(map_inq + INTC_MIR_SET3));
printk(KERN_EMERG "Register INTC_ISR_SET3 val = %X\n", __raw_readl(map_inq + INTC_ISR_SET3));
printk(KERN_EMERG "Register INTC_ISR_CLEAR3 val = %X\n", __raw_readl(map_inq + INTC_ISR_CLEAR3));
printk(KERN_EMERG "Register INTC_PENDING_IRQ3 val = %X\n", __raw_readl(map_inq + INTC_PENDING_IRQ3));
printk(KERN_EMERG "Register INTC_SYSCONFIG val = %X\n", __raw_readl(map_inq + INTC_SYSCONFIG));
return;
}
