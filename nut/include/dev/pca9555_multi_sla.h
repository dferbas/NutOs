
#ifndef _DEV_PCA9555_H_
#define	_DEV_PCA9555_H_

#include <cfg/pca9555.h>
#include <dev/twif.h>

__BEGIN_DECLS

int IOPCAReInit(uint8_t psla);
int IOPCASetOutput(uint8_t psla, uint8_t *data);
int IOPCAGetOutput(uint8_t psla, uint8_t *data);
int IOPCASetOutputBit(uint8_t psla, uint8_t bit, uint8_t value);
int IOPCAGetOutputBit(uint8_t psla, uint8_t bit, uint8_t *value);
int IOPCASetConfiguration(uint8_t psla, uint8_t *data);
int IOPCAGetConfiguration(uint8_t psla, uint8_t *data);
int IOPCASetInverter(uint8_t psla, uint8_t *data);
int IOPCAGetInverter(uint8_t psla, uint8_t *data);
int IOPCAGetInput(uint8_t psla, uint8_t *data );

__END_DECLS
/* End of prototypes */
#endif
