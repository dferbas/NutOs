#include <compiler.h>
#include <stdlib.h>
#include <string.h>
#include <memdebug.h>
#include <sys/heap.h>
#include <sys/event.h>

#include <cfg/os.h>
#include <dev/twif.h>

#include <dev/gpio.h>
#include <cfg/pca9555.h>
#include <dev/pca9555_multi_sla.h>

#define I2C_FIXED_SLA_IOPCA     0x20

#define PCA_PINP    0   /**< PCA Input register offset */
#define PCA_POUT    2   /**< PCA Output register offset */
#define PCA_PINV    4   /**< PCA Polarity inversion register offset */
#define PCA_CONF    6   /**< PCA Configuration register offset */

typedef struct __attribute__ ((packed))
{
	void *nextPCA9555;
    uint8_t output[2];
    uint8_t invert[2];
    uint8_t config[2];
    uint8_t slave_address;
} dcb_pca;

static dcb_pca *p_pca = NULL;

/*
 * Create new PCA9555 with programmable Slave address
 */
static dcb_pca *createPCA(uint8_t psla, dcb_pca *p_previous_pca){

	dcb_pca *new_pca = (dcb_pca*) NutHeapAlloc(sizeof(dcb_pca));
	if (p_previous_pca == NULL) {
		p_pca = new_pca;
	}
	else{
		p_previous_pca->nextPCA9555 = new_pca;
	}
	new_pca->slave_address = psla;
	new_pca->nextPCA9555 = NULL;

	// Init PCA before use
	IOPCAReInit(psla);

	return new_pca;

}

/*
 * Get PCA9555, if doesnt exist create new
 */
static dcb_pca *getPCA(uint8_t psla)
{
	dcb_pca *p_next_pca;
	dcb_pca *p_actual_pca;

	if(p_pca == NULL){
		p_actual_pca = createPCA(psla, NULL);
	}
	else{
		p_next_pca = p_pca;
		while(1){
			if(psla == p_next_pca->slave_address){
				p_actual_pca = p_next_pca;
				break;
			}
			if(p_next_pca->nextPCA9555 == NULL){
				p_actual_pca = createPCA(psla, p_next_pca);
				break;
			}
			p_next_pca = p_next_pca->nextPCA9555;
		}
	}

    return p_actual_pca;
}
    
/*
 * ReInit PCA9555, called when PCA9555 structure first used
 * \params: psla - programable slave address + I2C offset 0x80 for I2C0 or I2C1
 */
int IOPCAReInit(uint8_t psla)
{
	uint8_t data[2];

	//create PCA9555 if doesnt exist in IOPCASetConfiguration
    
	data[0] = 0xFF; // Set as Inputs
	data[1] = 0xFF;
	IOPCASetConfiguration(psla, data);

	data[0] = 0xFF; // Set as Inverted
	data[1] = 0xFF;
	IOPCASetInverter(psla, data);

	data[0] = 0x00; // Set outputs 0
	data[1] = 0x00;
	IOPCASetOutput(psla, data);

	return 0;
}

    

/*
 * Set Output port registers
 * \params: psla - programable slave address + I2C offset 0x80 for I2C0 or I2C1
 * 			data - size of data = 2
 */
int IOPCASetOutput(uint8_t psla, uint8_t *data)
{
	uint8_t reg_address = PCA_POUT;
	dcb_pca *p_actual_pca = getPCA(psla);

	p_actual_pca->output[0] = data[0];
	p_actual_pca->output[1] = data[1];
    
    if( TwMasterWrite( I2C_FIXED_SLA_IOPCA + psla, &reg_address, 1, p_actual_pca->output, 2, 50) == -1)
        return -1;

    return 0;      
}

/*
 * Set Output bit in port registers
 * \params: psla - programable slave address + I2C offset 0x80 for I2C0 or I2C1
 *
 */
int IOPCASetOutputBit(uint8_t psla, uint8_t bit, uint8_t value)
{
	uint8_t reg_address = PCA_POUT;
	uint8_t bank = 1;
	dcb_pca *p_actual_pca = getPCA(psla);

	if(bit < 8){
		bank = 0;
	}
	if (value) {
		p_actual_pca->output[bank] |= 0x01 << (bit % 8);
	} else {
		p_actual_pca->output[bank] &= ~(0x01 << (bit % 8));
	}

    if( TwMasterWrite( I2C_FIXED_SLA_IOPCA + psla, &reg_address, 1, p_actual_pca->output, 2, 50) == -1)
		return -1;

	return 0;
}

/*
 * Get Output port registers
 * \params: psla - programable slave address + I2C offset 0x80 for I2C0 or I2C1
 * 			data - size of data = 2
 */
int IOPCAGetOutput(uint8_t psla, uint8_t *data)
{
	dcb_pca *p_actual_pca = getPCA(psla);

	data[0] = p_actual_pca->output[0];
	data[1] = p_actual_pca->output[1];

	return 0;
}

/*
 * Get Output bit in port registers
 * \params: psla - programable slave address + I2C offset 0x80 for I2C0 or I2C1
 *
 */
int IOPCAGetOutputBit(uint8_t psla, uint8_t bit, uint8_t *value)
{
	uint8_t bank = 1;
	dcb_pca *p_actual_pca = getPCA(psla);

	if(bit < 8){
		bank = 0;
	}

	*value = (p_actual_pca->output[bank] & (0x01 << (bit % 8))) ? 1 : 0;

    return 0;
}

/*
 * Set Configuration registers
 * \params: psla - programable slave address + I2C offset 0x80 for I2C0 or I2C1
 * 			data - size of data = 2
 */
int IOPCASetConfiguration(uint8_t psla, uint8_t *data)
{
	uint8_t reg_address = PCA_CONF;

	//if PCA9555 does not exist, create new one
	dcb_pca *p_actual_pca = getPCA(psla);

	p_actual_pca->config[0] = data[0];
	p_actual_pca->config[1] = data[1];

	if( TwMasterWrite( I2C_FIXED_SLA_IOPCA + psla, &reg_address, 1, p_actual_pca->config, 2, 50) == -1)
		return -1;

	return 0;
}

/*
 * Get Configuration registers
 * \params: psla - programable slave address + I2C offset 0x80 for I2C0 or I2C1
 * 			data - size of data = 2
 */
int IOPCAGetConfiguration(uint8_t psla, uint8_t *data)
{
	dcb_pca *p_actual_pca = getPCA(psla);

	data[0] = p_actual_pca->config[0];
	data[1] = p_actual_pca->config[1];

	return 0;
}

/*
 * Set Polarity Inversion registers
 * \params: psla - programable slave address + I2C offset 0x80 for I2C0 or I2C1
 * 			data - size of data = 2
 */
int IOPCASetInverter(uint8_t psla, uint8_t *data)
{
	uint8_t reg_address = PCA_PINV;
	dcb_pca *p_actual_pca = getPCA(psla);

	p_actual_pca->invert[0] = data[0];
	p_actual_pca->invert[1] = data[1];

    if( TwMasterWrite( I2C_FIXED_SLA_IOPCA + psla, &reg_address, 1, p_actual_pca->invert, 2, 50) == -1)
		return -1;

    return 0;
}

/*
 * Get Polarity Inversion registers
 * \params: psla - programable slave address + I2C offset 0x80 for I2C0 or I2C1
 * 			data - size of data = 2
 */
int IOPCAGetInverter(uint8_t psla, uint8_t *data)
{
	dcb_pca *p_actual_pca = getPCA(psla);

	data[0] = p_actual_pca->invert[0];
	data[1] = p_actual_pca->invert[1];

	return 0;
}

/*
 * Get Input registers
 * \params: psla - programable slave address + I2C offset 0x80 for I2C0 or I2C1
 * 			data - size of data = 2
 */
int IOPCAGetInput(uint8_t psla, uint8_t *data )
{
	uint8_t reg_address = PCA_PINP;
	// Initialization if not Initialized
	getPCA(psla);

	if( TwMasterRead( I2C_FIXED_SLA_IOPCA + psla, &reg_address, 1, data, 2, 50) == -1)
		return-1;
	return 0;
}


