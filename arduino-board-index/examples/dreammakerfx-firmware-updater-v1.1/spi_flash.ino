#include <SPI.h>

// GPIO lines used in the flashing process
#define SHARC_SPI_SELECT        (7)   
#define SHARC_RESET             (6)   

// Defines for the SPI flash memory
#define CMD_SPI_READ            (0x03)
#define CMD_SPI_PROG_PAGE       (0x02)
#define CMD_SPI_SECTOR_ERASE    (0x20)
#define CMD_SPI_BLOCK_ERASE     (0xD8)
#define CMD_SPI_CHIP_ERASE      (0xC7)
#define CMD_SPI_READ_STATUS     (0x05)
#define CMD_SPI_WRITE_STATUS    (0x05)
#define CMD_SPI_WRITE_EN        (0x06)
#define CMD_SPI_WRITE_EN_NV     (0x50)


#define SPI_STATUS_BUSY         (0x1)
#define SPI_STATUS_WRITE_EN     (0x2)


/**
 * @brief Asserts the reset line on the DSP
 * 
 */
void  dsp_assert_reset(void) {
  digitalWrite(SHARC_RESET, LOW);
  delay(1);
}

/**
 * @brief Deasserts the reset line on the DSP
 */
void  dsp_deassert_reset(void) {
  digitalWrite(SHARC_RESET, HIGH);
}

/**
 * @brief Starts a SPI transfer
 */
void  spi_flash_start_transfer(void) {

    // SHARC bootloader defaults to LSB first
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SHARC_SPI_SELECT, LOW);
}

/**
 * @brief Ends a SPI transfer
 */
void  spi_flash_end_transfer(void) {
 
    SPI.endTransaction();
    digitalWrite(SHARC_SPI_SELECT, HIGH);
    delay(1);
}

/**
 * @brief Reads the status register in the spi flash
 */
uint8_t spi_flash_read_status_register() {
    
    uint8_t result;
    
    spi_flash_start_transfer();
    
    SPI.transfer(CMD_SPI_READ_STATUS);   
    result = SPI.transfer(0x0);  
        
    spi_flash_end_transfer();
    
    return result;

}

/**
 * @brief Sends a single byte command to the flash
 */
static void spi_flash_send_byte(uint8_t val) {
   
    // Send a single byte command
    spi_flash_start_transfer();
    SPI.transfer(val);   
    spi_flash_end_transfer();
}

/**
 * @brief Checks to see if the SPI flash is busy
 */
static bool spi_flash_check_busy(void) {
    
    uint8_t val = spi_flash_read_status_register();
    
    if (val & SPI_STATUS_BUSY) return true;
    else return false;
}


/**
 * @brief Erases the entire flash memory
 */
static void    spi_flash_erase_chip(void) {

    Serial.print(" SPI Flash: erasing flash...");
    spi_flash_send_byte(CMD_SPI_WRITE_EN);
    spi_flash_send_byte(CMD_SPI_CHIP_ERASE);
    while (spi_flash_check_busy());
    Serial.println(" complete");

}

/**
 * @brief Not sure if needed but clears the non-volatile 
 * write protection bits in the event that they get set
 */
static void   spi_flash_clear_protect(void) {

    spi_flash_start_transfer();
    SPI.transfer(CMD_SPI_WRITE_EN_NV);
    spi_flash_end_transfer();


    spi_flash_start_transfer();
    
    SPI.transfer(CMD_SPI_WRITE_STATUS);   
    SPI.transfer(0x0);  
        
    spi_flash_end_transfer();
  
}


/**
 * @brief flips the bit order of a byte.  The DSP defaults to LSB first but the 
 * SPI commands need to be MSB first.  Since we can't change the interface
 * mid command, we need to manually flip the bits.  No worries though, we have all
 * day.
 */
uint8_t flip_bit_order(uint8_t num) 
{ 
    unsigned int reverse_num = 0; 
    int i; 
    for (i=0; i<8; i++) 
    { 
        if((num & (1 << i))) 
           reverse_num |= 1 << ((8 - 1) - i);   
   } 
   return reverse_num; 
} 

/**
 * @brief Writes a 256 byte page to flash memory
 */
static bool    spi_flash_page_write(uint32_t address, uint8_t * vals, uint16_t count) {
    
    if (count > 256) {
        return false;
    }
    
    uint8_t spi_block[4] = {CMD_SPI_PROG_PAGE};
    
    // Load address
    spi_block[1] = (address >> 16) & 0xFF;
    spi_block[2] = (address >> 8) & 0xFF;
    spi_block[3] = (address >> 0) & 0xFF;


    spi_flash_send_byte(CMD_SPI_WRITE_EN);   
     
    spi_flash_start_transfer();
    for (int i=0;i<4;i++) {
       SPI.transfer(spi_block[i]);  
    }
    for (int i=0;i<count;i++) {
      SPI.transfer(flip_bit_order(vals[i]));        
    }
    spi_flash_end_transfer();    
    
    // 3. Wait for transaction to complete
    while (spi_flash_check_busy());
        
    return true;
}

/**
 * @brief Flashes the DSP boot flash and then resets the DSP
 */
bool spi_flash_program(uint8_t * vals, uint32_t count) {


  Serial.println("SPI Flash: initializing SPI flash interface...");

  // Set up SPI select pin
  pinMode(SHARC_SPI_SELECT, OUTPUT); 
  digitalWrite(SHARC_SPI_SELECT, HIGH); 

  pinMode(SHARC_RESET, OUTPUT); 
  digitalWrite(SHARC_RESET, HIGH); 


  // Start SPI port
  SPI.begin();  

  dsp_assert_reset();

  spi_flash_clear_protect();
  spi_flash_erase_chip();

  uint32_t page_count = (count >> 8) + 1;
  uint32_t address = 0;

  Serial.println(" SPI Flash: programming");

  uint32_t byte_cnt = 0;
  for (int i=0;i<page_count;i++) {    
    spi_flash_page_write(address, &vals[address], 256); 
    address += 256;
  }

  Serial.print("SPI Flash: complete; wrote ");
  Serial.print(address/1024);
  Serial.println(" kBytes");

  // Turn SPI select back into input so SHARC can boot
  pinMode(SHARC_SPI_SELECT, INPUT); 
  SPI.end(); 

  Serial.print("Resetting DSP");
  delay(10);
  dsp_deassert_reset();
  delay(10);
  dsp_assert_reset();
  delay(10);
  dsp_deassert_reset();

  return true;
  
}
