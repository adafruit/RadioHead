// RHEncryptedDriver.h

// Generic encryption layer that could use any driver
// But will encrypt all data.
// Requires the Arduinolibs/Crypto library:
// https://github.com/rweather/arduinolibs
//
// Author: Philippe.Rochat'at'gmail.com
// Contributed to the RadioHead project by the author
// $Id: RadioHead.h,v 1.65 2017/06/25 09:41:17 mikem Exp $

#ifndef RHEncryptedDriver_h
#define RHEncryptedDriver_h

#include <RHGenericDriver.h>
#ifdef RH_ENABLE_ENCRYPTION_MODULE
#include <BlockCipher.h>

// Undef this if trailing 0 on each enrypted message is ok.
// This defined means a first byte of the payload is used to encode content length
// And the received message content is trimmed to this length
#define STRICT_CONTENT_LEN  

// Define this to allow encrypted content to span over 2 messages
// STRICT_CONTENT_LEN and ALLOW_MULTIPLE_MSG aren't compatible !!!
// With STRICT_CONTENT_LEN, receiver will try to extract length from every message !!!!
//#define ALLOW_MULTIPLE_MSG  

/////////////////////////////////////////////////////////////////////
/// \class RHEncryptedDriver RHEncryptedDriver <RHEncryptedDriver.h>
/// \brief Virtual Driver to encrypt/decrypt sent data. Could be used with any other driver.
///
/// This driver acts as a wrapper for any other RadioHead driver, adding encryption and decryption of
/// messages that are passed to and from the actual radio driver. Any of the encryption ciphers supported by
/// ArduinoLibs Cryptographic Library http://rweather.github.io/arduinolibs/crypto.html may be used.
///
/// For successful communications, both sender and receiver must use the same cipher and the same key.
///
/// Just call the constructor with your real radio modem and a BlockCiphering class from the
/// Arduinolibs: https://github.com/rweather/arduinolibs.
///
/// In order to enable this module you must uncomment #define RH_ENABLE_ENCRYPTION_MODULE at the bottom of RadioHead.h
/// But ensure you have installed the Crypto directory from arduinolibs first:
/// http://rweather.github.io/arduinolibs/index.html

class RHEncryptedDriver : public RHGenericDriver
{
public:
    /// Constructor.
    /// Adds a ciphering layer to messages sent and received by the actual transport driver.
    /// \param[in] driver The RadioHead driver to use to transport messages.
    /// \param[in] blockcipher The blockcipher (from arduinolibs) that crypt/decrypt data. Ensure that
    /// the blockcipher has had its key set before sending or receiving messages.
    RHEncryptedDriver(RHGenericDriver& driver, BlockCipher& blockcipher);
	
    /// Tests whether a new message is available
    /// from the Driver. 
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received by the transport, when it wil be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool available();

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    virtual bool recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then optionally waits for Channel Activity Detection (CAD) 
    /// to show the channnel is clear (if the radio supports CAD) by calling waitCAD().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send
    /// specify the maximum time in ms to wait. If 0 (the default) do not wait for CAD before transmitting.
    /// \return true if the message length was valid and it was correctly queued for transmit. Return false
    /// if CAD was requested and the CAD timeout timed out before clear channel was detected.
    virtual bool send(const uint8_t* data, uint8_t len);

    /// Blocks until the transmitter 
    /// is no longer transmitting.
    virtual bool            waitPacketSent();

    /// Returns the maximum message length 
    /// available in this Driver, which depends on the maximum length supported by the underlying transport driver.
    /// \return The maximum legal message length
    virtual  uint8_t maxMessageLength();


private:
    /// The underlying transport river we are to use
    RHGenericDriver&        _driver;
    
    /// The CipherBlock we are to use for encrypting/decrypting
    BlockCipher&	    _blockcipher;
    
    /// Struct for with buffers for ciphering
    typedef struct
    {
	size_t  blockSize    = 0;
	uint8_t *inputBlock  = NULL;
	//uint8_t *outputBlock = NULL;		
    } CipherBlocks;
    
    CipherBlocks            _cipheringBlocks;
    
    /// Buffer to store encrypted/decrypted message
    uint8_t*                _buffer;
};

/// @example nrf24_encrypted_client.pde
/// @example nrf24_encrypted_server.pde
/// @example rf95_encrypted_client.pde
/// @example rf95_encrypted_server.pde


#endif
#endif
