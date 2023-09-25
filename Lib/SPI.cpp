/*
 * MIT License
 * Copyright (c) 2019 _VIFEXTech
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "SPI.h"

#define SPI1_CLOCK (F_CPU)
#define SPI2_CLOCK (F_CPU/2)
#define SPI3_CLOCK (F_CPU/2)

SPIClass::SPIClass(SPI_TypeDef* _SPIx)
{
    SPIx = _SPIx;
}

void SPIClass::SPI_Settings(    SPI_TypeDef* SPIx,
                                uint16_t SPI_Mode_x,
                                uint16_t SPI_DataSize_x,
                                uint16_t SPI_MODEx,
                                uint16_t SPI_NSS_x,
                                uint16_t SPI_BaudRatePrescaler_x,
                                uint16_t SPI_FirstBit_x)
{
    uint16_t SPI_CPOL_x, SPI_CPHA_x;
    LL_SPI_Disable(SPIx);

    switch(SPI_MODEx)
    {
    case 0:
        SPI_CPOL_x = LL_SPI_POLARITY_LOW;
        SPI_CPHA_x = LL_SPI_PHASE_1EDGE;
        break;
    case 1:
        SPI_CPOL_x = LL_SPI_POLARITY_LOW;
        SPI_CPHA_x = LL_SPI_PHASE_2EDGE;
        break;
    case 2:
        SPI_CPOL_x = LL_SPI_POLARITY_HIGH;
        SPI_CPHA_x = LL_SPI_PHASE_1EDGE;
        break;
    case 3:
        SPI_CPOL_x = LL_SPI_POLARITY_HIGH;
        SPI_CPHA_x = LL_SPI_PHASE_2EDGE;
        break;
    }

    SPI_InitStructure.TransferDirection = LL_SPI_FULL_DUPLEX;  //设置SPI单向或者双向的数据模式
    SPI_InitStructure.Mode = SPI_Mode_x;        //设置SPI工作模式:(SPI_Mode_Master,SPI_Mode_Slave)
    SPI_InitStructure.DataWidth = SPI_DataSize_x;        //设置SPI的数据大小:SPI发送接收x位帧结构
    SPI_InitStructure.ClockPolarity = SPI_CPOL_x;        //选择了串行时钟的稳态
    SPI_InitStructure.ClockPhase = SPI_CPHA_x;    //数据捕获时钟沿
    SPI_InitStructure.NSS = SPI_NSS_x;      //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制(SPI_NSS_Soft,SPI_NSS_Hard)
    SPI_InitStructure.BaudRate = SPI_BaudRatePrescaler_x;      //定义波特率预分频的值
    SPI_InitStructure.BitOrder = SPI_FirstBit_x;    //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始(SPI_FirstBit_MSB,SPI_FirstBit_LSB)
    SPI_InitStructure.CRCPoly = 3;    //CRC值计算的多项式
    LL_SPI_Init(SPIx, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

    LL_SPI_Enable(SPIx);  //使能SPI外设
}

void SPIClass::begin(void)
{
    if(SPIx == SPI1)
    {
        SPI_Clock = SPI1_CLOCK;
//        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
        pinMode(PA5, OUTPUT_AF);
        pinMode(PA6, OUTPUT_AF);
        pinMode(PA7, OUTPUT_AF);

//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

//        RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);
//        RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);
    }
    else if(SPIx == SPI2)
    {
        SPI_Clock = SPI2_CLOCK;
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
        pinMode(PB13, OUTPUT_AF);
        pinMode(PB14, OUTPUT_AF);
        pinMode(PB15, OUTPUT_AF);

//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

//        RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);
//        RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, DISABLE);
    }
    else if(SPIx == SPI3)
    {
        SPI_Clock = SPI3_CLOCK;
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
        pinMode(PB3, OUTPUT_AF);
        pinMode(PB4, OUTPUT_AF);
        pinMode(PB5, OUTPUT_AF);

//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);

//        RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
//        RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);
    }
    else
    {
        return;
    }

    SPI_Settings(   SPIx,
                    LL_SPI_MODE_MASTER,
                    LL_SPI_DATAWIDTH_8BIT,
                    SPI_MODE0,
                    LL_SPI_NSS_HARD_INPUT,
                    LL_SPI_BAUDRATEPRESCALER_DIV2,
                    LL_SPI_MSB_FIRST);
}

void SPIClass::begin(uint32_t clock, uint16_t dataOrder, uint16_t dataMode)
{
    begin();
    setClock(clock);
    setBitOrder(dataOrder);
    setDataMode(dataMode);
    LL_SPI_Enable(SPIx);  //使能SPI外设
}

void SPIClass::begin(SPISettings settings)
{
    begin();
    setClock(settings.clock);
    setBitOrder(settings.bitOrder);
    setDataMode(settings.dataMode);
    LL_SPI_Enable(SPIx);  //使能SPI外设
}

void SPIClass::beginSlave(void)
{
    begin();
    SPI_Settings(   SPIx,
                    LL_SPI_MODE_SLAVE,
                    LL_SPI_DATAWIDTH_8BIT,
                    SPI_MODE0,
                    LL_SPI_NSS_HARD_INPUT,
                    LL_SPI_BAUDRATEPRESCALER_DIV16,
                    LL_SPI_MSB_FIRST);
    LL_SPI_Enable(SPIx);  //使能SPI外设
}

void SPIClass::end(void)
{
    LL_SPI_Disable(SPIx);
}

void SPIClass::setClock(uint32_t clock)
{
    uint16_t SPI_BaudRatePrescaler_x;
    uint16_t clock_div = SPI_Clock / clock;
    if(clock_div <= 2)SPI_BaudRatePrescaler_x = LL_SPI_BAUDRATEPRESCALER_DIV2;
    else if(clock_div <= 4)SPI_BaudRatePrescaler_x = LL_SPI_BAUDRATEPRESCALER_DIV4;
    else if(clock_div <= 8)SPI_BaudRatePrescaler_x = LL_SPI_BAUDRATEPRESCALER_DIV8;
    else if(clock_div <= 16)SPI_BaudRatePrescaler_x = LL_SPI_BAUDRATEPRESCALER_DIV16;
    else if(clock_div <= 32)SPI_BaudRatePrescaler_x = LL_SPI_BAUDRATEPRESCALER_DIV32;
    else if(clock_div <= 64)SPI_BaudRatePrescaler_x = LL_SPI_BAUDRATEPRESCALER_DIV64;
    else if(clock_div <= 128)SPI_BaudRatePrescaler_x = LL_SPI_BAUDRATEPRESCALER_DIV128;
    else SPI_BaudRatePrescaler_x = LL_SPI_BAUDRATEPRESCALER_DIV256;
    SPI_InitStructure.BaudRate = SPI_BaudRatePrescaler_x;
    LL_SPI_Init(SPIx, &SPI_InitStructure);
    LL_SPI_Enable(SPIx);  //使能SPI外设
}

void SPIClass::setClockDivider(uint32_t Div)
{
    setClock(SPI_Clock / Div);
}

void SPIClass::setBitOrder(uint16_t bitOrder)
{
    if(bitOrder == MSBFIRST)SPI_InitStructure.BitOrder = LL_SPI_MSB_FIRST;//MSBFIRST 1
    else SPI_InitStructure.BitOrder = LL_SPI_LSB_FIRST;
    LL_SPI_Init(SPIx, &SPI_InitStructure);
    LL_SPI_Enable(SPIx);  //使能SPI外设
}

/*  Victor Perez. Added to test changing datasize from 8 to 16 bit modes on the fly.
*   Input parameter should be SPI_CR1_DFF set to 0 or 1 on a 32bit word.
*
*/
void SPIClass::setDataSize(uint16_t datasize)
{
    SPI_InitStructure.DataWidth = datasize;
    LL_SPI_Init(SPIx, &SPI_InitStructure);
    LL_SPI_Enable(SPIx);  //使能SPI外设
}

void SPIClass::setDataMode(uint8_t dataMode)
{
    /* Notes.  As far as I can tell, the AVR numbers for dataMode appear to match the numbers required by the STM32

    From the AVR doc http://www.atmel.com/images/doc2585.pdf section 2.4

    SPI Mode    CPOL    CPHA    Shift SCK-edge  Capture SCK-edge
    0           0       0       Falling         Rising
    1           0       1       Rising          Falling
    2           1       0       Rising          Falling
    3           1       1       Falling         Rising


    On the STM32 it appears to be

    bit 1 - CPOL : Clock polarity
        (This bit should not be changed when communication is ongoing)
        0 : CLK to 0 when idle
        1 : CLK to 1 when idle

    bit 0 - CPHA : Clock phase
        (This bit should not be changed when communication is ongoing)
        0 : The first clock transition is the first data capture edge
        1 : The second clock transition is the first data capture edge

    If someone finds this is not the case or sees a logic error with this let me know ;-)
     */
    uint16_t SPI_CPOL_x, SPI_CPHA_x;
    LL_SPI_Disable(SPIx);  

    switch(dataMode)
    {
    case SPI_MODE0:
        SPI_CPOL_x = LL_SPI_POLARITY_LOW;
        SPI_CPHA_x = LL_SPI_PHASE_1EDGE;
        break;
    case SPI_MODE1:
        SPI_CPOL_x = LL_SPI_POLARITY_LOW;
        SPI_CPHA_x = LL_SPI_PHASE_2EDGE;
        break;
    case SPI_MODE2:
        SPI_CPOL_x = LL_SPI_POLARITY_HIGH;
        SPI_CPHA_x = LL_SPI_PHASE_1EDGE;
        break;
    case SPI_MODE3:
        SPI_CPOL_x = LL_SPI_POLARITY_HIGH;
        SPI_CPHA_x = LL_SPI_PHASE_2EDGE;
        break;
    }

    SPI_InitStructure.ClockPolarity = SPI_CPOL_x;
    SPI_InitStructure.ClockPhase = SPI_CPHA_x;
    LL_SPI_Init(SPIx, &SPI_InitStructure);
    LL_SPI_Enable(SPIx);  //使能SPI外设
}

void SPIClass::beginTransaction(SPISettings settings)
{
    SPISettings(settings.clock, settings.bitOrder, settings.dataMode);

    setClock(settings.clock);
    setBitOrder(settings.bitOrder);
    setDataMode(settings.dataMode);
    setDataSize(settings.dataSize);

    LL_SPI_Enable(SPIx);  //使能SPI外设
}

void SPIClass::beginTransactionSlave(void)
{
    beginSlave();
}

void SPIClass::endTransaction(void)
{
    LL_SPI_Disable(SPIx); 
}

#define SPI_I2S_GET_FLAG(SPI_I2S_FLAG) (SPIx->SR & SPI_I2S_FLAG)
#define SPI_I2S_RXDATA()               (SPIx->DR)
#define SPI_I2S_RXDATA_VOLATILE()      volatile uint16_t vn = SPI_I2S_RXDATA()
#define SPI_I2S_TXDATA(data)           (SPIx->DR = data)

uint16_t SPIClass::read(void)
{
    while (!SPI_I2S_GET_FLAG(LL_SPI_SR_RXNE));
    return (uint16_t)SPI_I2S_RXDATA();
}

void SPIClass::read(uint8_t *buf, uint32_t len)
{
    if (len == 0)
        return;

    SPI_I2S_RXDATA_VOLATILE();
    SPI_I2S_TXDATA(0x00FF);

    while((--len))
    {
        while (!SPI_I2S_GET_FLAG(LL_SPI_SR_TXE));
        noInterrupts();
        SPI_I2S_TXDATA(0x00FF);
        while (!SPI_I2S_GET_FLAG(LL_SPI_SR_RXNE));
        *buf++ = (uint8_t)SPI_I2S_RXDATA();
        interrupts();
    }
    while (!SPI_I2S_GET_FLAG(LL_SPI_SR_RXNE));
    *buf++ = (uint8_t)SPI_I2S_RXDATA();
}

void SPIClass::write(uint16_t data)
{
    SPI_I2S_TXDATA(data);
    while (!SPI_I2S_GET_FLAG(LL_SPI_SR_TXE));
    while SPI_I2S_GET_FLAG(LL_SPI_SR_BSY);
}

void SPIClass::write(uint16_t data, uint32_t n)
{
    while ((n--) > 0)
    {
        SPI_I2S_TXDATA(data); // write the data to be transmitted into the SPI_DR register (this clears the TXE flag)
        while (SPI_I2S_GET_FLAG(SPI_SR_TXE) == 0); // wait till Tx empty
    }

    while (SPI_I2S_GET_FLAG(SPI_SR_BSY) != 0); // wait until BSY=0 before returning
}

void SPIClass::write(const uint8_t *data, uint32_t length)
{
    while (length--)
    {
        while (SPI_I2S_GET_FLAG(SPI_SR_TXE) == 0);
        SPI_I2S_TXDATA(*data++);
    }
    while (!SPI_I2S_GET_FLAG(SPI_SR_TXE));
    while (SPI_I2S_GET_FLAG(SPI_SR_BSY));
}

void SPIClass::write(const uint16_t *data, uint32_t length)
{
    while (length--)
    {
        while (SPI_I2S_GET_FLAG(SPI_SR_TXE) == 0);
        SPI_I2S_TXDATA(*data++);
    }
    while (!SPI_I2S_GET_FLAG(SPI_SR_TXE));
    while (SPI_I2S_GET_FLAG(SPI_SR_BSY));
}

uint8_t SPIClass::transfer(uint8_t wr_data) const
{
    SPI_I2S_RXDATA_VOLATILE();
    SPI_I2S_TXDATA(wr_data);
    while (!SPI_I2S_GET_FLAG(LL_SPI_SR_TXE));
    while SPI_I2S_GET_FLAG(LL_SPI_SR_BSY);
    return (uint8_t)SPI_I2S_RXDATA();
}

uint16_t SPIClass::transfer16(uint16_t wr_data) const
{
    SPI_I2S_RXDATA_VOLATILE();
    SPI_I2S_TXDATA(wr_data);
    while (!SPI_I2S_GET_FLAG(LL_SPI_SR_TXE));
    while SPI_I2S_GET_FLAG(LL_SPI_SR_BSY);
    return (uint16_t)SPI_I2S_RXDATA();
}

uint8_t SPIClass::send(uint8_t data)
{
    this->write(data);
    return 1;
}

uint8_t SPIClass::send(uint8_t *buf, uint32_t len)
{
    this->write(buf, len);
    return len;
}

uint8_t SPIClass::recv(void)
{
    return this->read();
}

SPIClass SPI(SPI1);//SCK-PA5 MISO-PA6 MOSI-PA7
SPIClass SPI_2(SPI2);//SCK-PB13 MISO-PB14 MOSI-PB15
SPIClass SPI_3(SPI3);//SCK-PB3 MISO-PB4 MOSI-PB5
