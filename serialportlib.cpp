#include "serialportlib.h"

#include <cstdint>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cerrno>

#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

typedef uint32_t SP_IDX;

#define MAX_SERIALPORTS (SERIALPORTLIB_MAX_SERIALPORT_IDX + 1)

#define INFO "SERIALPORTLIB INFO: "
#define WARNING "SERIALPORTLIB WARNING: "
#define ERROR "SERIALPORTLIB ERROR: "

struct Serialport {
    char* portname = nullptr;
    bool connected = false;
    int fd = -1;

    // int delimiter_length = 0;
    // uint8_t delimiter[SERIALPORTLIB_DELIMITER_MAX_BYTES] = {};

    // int leftovers_length = 0;
    // uint8_t leftover_bytes[SERIALPORTLIB_OBJECT_MAX_BYTES + SERIALPORTLIB_DELIMITER_MAX_BYTES] = {};
};

static struct {
    Serialport serialports[MAX_SERIALPORTS];
} gsps; // Global-SerialPortlib-State

static bool valid_serialport_idx(SP_IDX sp_idx) {
    if (sp_idx <= SERIALPORTLIB_MAX_SERIALPORT_IDX) return true;
    printf(ERROR "The serialport-index '%u' is not within the valid fixed range of [0, %d]\n", sp_idx, SERIALPORTLIB_MAX_SERIALPORT_IDX);
    return false;
}

static bool valid_idx_and_connected(SP_IDX sp_idx) {
    if (!valid_serialport_idx(sp_idx))        return false;
    if (gsps.serialports[sp_idx].connected)   return true;
    printf(ERROR "The serialport at index '%u' is not connected.\n", sp_idx);
    return false;
}

// Based on: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
SPAPI bool serialport_open(uint32_t sp_idx, const char* portname, uint32_t baudrate)
{
    if (!valid_serialport_idx(sp_idx)) return false;
    
    Serialport& sp = gsps.serialports[sp_idx];
    
    if (strlen(portname) == 0) {
        printf(ERROR "Faild to connect. No portname was specified.");
        return false;
    }

    size_t portname_len = strlen(portname) + 1;
    delete [] sp.portname;
    sp.portname = new char[portname_len];
    strncpy(sp.portname, portname, portname_len);
    
    if (sp.connected) {
        serialport_close(sp_idx);
    }

    sp.fd = open(portname, O_RDWR | O_NONBLOCK);
    if (sp.fd < 0) {
        printf(ERROR "Faild to open serialport at '%s'. errno (%d): %s\n", portname, errno, strerror(errno));
        return false;
    }

    // TERMIOS configuration

    termios port_settings;
    if(tcgetattr(sp.fd, &port_settings) != 0) {
        printf(ERROR "tcgetattr() failed to get attributes of the port at %s. errno (%d): %s\n", portname, errno, strerror(errno));
        serialport_close(sp_idx);
        return false;
    }

    port_settings.c_cflag = 0x0;
    port_settings.c_cflag |= CS8;            // Settings size to 8 bits per byte
    port_settings.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    port_settings.c_lflag = 0x0; // Disable all
    
    port_settings.c_iflag = 0x0; // Disable all

    port_settings.c_oflag = 0x0; // Disable all

    port_settings.c_cc[VTIME] = 0; // Just return all available bytes when read() and none when there are none (don't wait)
    port_settings.c_cc[VMIN] = 0;

    // TODO: test if we can actually set an arbitrary baudrate
    cfsetispeed(&port_settings, baudrate); // Baudrate (in)
    cfsetospeed(&port_settings, baudrate); // (out)

    // Apply settings. This is permanent (changes still persist when the file_handle is closed or when our process terminates)
    if (tcsetattr(sp.fd, TCSANOW, &port_settings) != 0) {
        printf(ERROR "Failed to apply configuration to port '%s'. errno (%d): '%s'\n", portname, errno, strerror(errno));
        serialport_close(sp_idx);
        return false;
    }

    // IOCTL configuration

    int ioctl_flags;
    if (ioctl(sp.fd, TIOCMGET, &ioctl_flags) < 0) {
        printf(ERROR "Failed to get ioctl configuration from port '%s'. errno (%d): '%s'\n", portname, errno, strerror(errno));
        serialport_close(sp_idx);
        return false;
    }
    printf("Flags are %x.\n", ioctl_flags);

    ioctl_flags &= ~TIOCM_DTR; // disable dtr (Data Terminal Ready)
    ioctl_flags &= ~TIOCM_RTS; // disable rts (Request To Send)

    if (ioctl(sp.fd, TIOCMSET, &ioctl_flags) < 0) {
        printf(ERROR "Failed to apply ioctl configuration to port '%s'. errno (%d): '%s'\n", portname, errno, strerror(errno));
        serialport_close(sp_idx);
        return false;
    }

    printf(INFO "Successfully connected to port '%s'\n", sp.portname);

    sp.connected = true;
    return true;
}

static void set_flag(int& flags, int flag, bool value)
{
    if (value) {
        flags |= flag;
    }
    else {
        flags &= ~flag;
    }
}

SPAPI bool serialport_set_flag(SP_IDX sp_idx, Serial_Port_Flags flag, bool value)
{
    switch (flag) {
    case SERIALPORT_DTR:
    case SERIALPORT_RTS:
    {
        if (!valid_serialport_idx(sp_idx)) return false;
    
        Serialport& sp = gsps.serialports[sp_idx];
    
        int ioctl_flags;
        if (ioctl(sp.fd, TIOCMGET, &ioctl_flags) < 0) {
            printf(ERROR "Failed to get ioctl flags from port '%s'. errno (%d): '%s'\n", sp.portname, errno, strerror(errno));
            return 0;
        }

        switch (flag) {
        case SERIALPORT_DTR: set_flag(ioctl_flags, TIOCM_DTR, value); break;
        case SERIALPORT_RTS: set_flag(ioctl_flags, TIOCM_RTS, value); break;
        }

        if (ioctl(sp.fd, TIOCMSET, &ioctl_flags) < 0) {
            printf(ERROR "Failed to apply ioctl configuration to port '%s'. errno (%d): '%s'\n", sp.portname, errno, strerror(errno));
            serialport_close(sp_idx);
            return false;
        }

    } break;
    }
    
    return true;
}

SPAPI int serialport_set_flag(SP_IDX sp_idx)
{
    if (!valid_serialport_idx(sp_idx)) return false;
    
    Serialport& sp = gsps.serialports[sp_idx];
    
    int ioctl_flags;
    if (ioctl(sp.fd, TIOCMGET, &ioctl_flags) < 0) {
        printf(ERROR "Failed to get ioctl flags from port '%s'. errno (%d): '%s'\n", sp.portname, errno, strerror(errno));
        return 0;
    }

    return ioctl_flags;
}

SPAPI bool serialport_set_ioctl_flags(SP_IDX sp_idx, int ioctl_flags)
{
    if (!valid_serialport_idx(sp_idx)) return false;
    
    Serialport& sp = gsps.serialports[sp_idx];

    if (ioctl(sp.fd, TIOCMSET, &ioctl_flags) < 0) {
        printf(ERROR "Failed to apply ioctl flags to port '%s'. errno (%d): '%s'\n", sp.portname, errno, strerror(errno));
        return false;
    }

    usleep(1000);

    if (ioctl(sp.fd, TIOCMGET, &ioctl_flags) < 0) {
        printf(ERROR "Failed to get ioctl flags from port '%s'. errno (%d): '%s'\n", sp.portname, errno, strerror(errno));
        return 0;
    }

    printf(INFO "ioctl-flags set to 0x%x.\n", ioctl_flags);

    return true;
}

SPAPI bool serialport_is_open(SP_IDX sp_idx)
{
    if (!valid_serialport_idx(sp_idx)) return false;
    
    return gsps.serialports[sp_idx].connected;
}

SPAPI bool serialport_close(SP_IDX sp_idx)
{
    if (!valid_serialport_idx(sp_idx)) return false;
    
    Serialport& sp = gsps.serialports[sp_idx];
    close(sp.fd);
    sp.connected = false;
    
    return true;
}

SPAPI uint64_t serialport_bytes_available_to_read(uint32_t sp_idx)
{
    if (!valid_serialport_idx(sp_idx)) return 0;
    
    Serialport& sp = gsps.serialports[sp_idx];
    
    int bytes_available;
    if (ioctl(sp.fd, FIONREAD, &bytes_available) < 0) {
        printf(ERROR "Failed to get info on the number of bytes available for reading '%s'. errno (%d): %s\n",
               sp.portname, errno, strerror(errno));
        return 0;
    }

    return bytes_available;
}

SPAPI uint8_t serialport_read_byte(SP_IDX sp_idx)
{
    if(!valid_idx_and_connected(sp_idx)) return false;
    
    Serialport& sp = gsps.serialports[sp_idx];

    uint8_t byte;
    ssize_t bytes_read = read(sp.fd, &byte, 1);
    if (bytes_read < 0) {
        printf(ERROR "Failed to from the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
        return 0;
    }
    
    return byte;
}

SPAPI uint64_t serialport_read_bytes(SP_IDX sp_idx, void* bytes_buffer, uint64_t buffer_size)
{
    if(!valid_idx_and_connected(sp_idx)) return false;
    
    Serialport& sp = gsps.serialports[sp_idx];

    ssize_t bytes_read = read(sp.fd, bytes_buffer, buffer_size);
    if (bytes_read < 0) {
        printf(ERROR "Faild to read from the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
        return 0;
    }
    
    return bytes_read;
}

// SPAPI bool serialport_set_delimiter(SP_IDX sp_idx, void* delimiter, uint64_t delimiter_size)
// {
//     if (delimiter_size > SERIALPORTLIB_DELIMITER_MAX_BYTES) {
//         printf(ERROR "Your delimiter of size '%lu' is larger than the maximum of '%d'\n", delimiter_size, SERIALPORTLIB_DELIMITER_MAX_BYTES);
//         return false;
//     }
//     if (!valid_serialport_idx(sp_idx)) return false;
//     Serialport& sp = gsps.serialports[sp_idx];

//     sp.delimiter_length = delimiter_size;
//     memcpy(sp.delimiter, delimiter, delimiter_size);

//     std::stringstream delimiter_ss;
//     for (uint64_t i = 0; i < delimiter_size; ++i) {
//         delimiter_ss << std::hex << (int) ((uint8_t*) delimiter)[i];
//     }
//     printf(INFO "Serialport delimiter was set to: 0x%s\n", delimiter_ss.str().c_str());
//     return true;
// }

SPAPI bool serialport_skip_to_next_delimiter(SP_IDX sp_idx, void* delimiter, uint64_t delimiter_size)
{
    if (!valid_idx_and_connected(sp_idx)) return false;

    Serialport& sp = gsps.serialports[sp_idx];

    uint8_t* delim_cmp_buff = new uint8_t[delimiter_size];

    while (true)
    {
        // shift all bytes in the delim_cmp_buff to the left
        for (uint64_t i = 0; i < delimiter_size - 1; ++i) {
            delim_cmp_buff[i] = delim_cmp_buff[i + 1];
        }
        
        // read the next byte into the delim_cmp_buff
        ssize_t bytes_read = read(sp.fd, &delim_cmp_buff[delimiter_size - 1], 1);

        if (bytes_read < 0) {
            printf(ERROR "Faild to read from the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
            return false;
        }
        
        // compare the last 'delimiter_length' bytes with the delimiter
        if (memcmp(delim_cmp_buff, delimiter, delimiter_size) == 0) {
            return true;
        }
    }

    delete[] delim_cmp_buff;
}

SPAPI bool serialport_write_byte(uint32_t sp_idx, uint8_t byte)
{
    if (!valid_idx_and_connected(sp_idx)) return false;

    Serialport& sp = gsps.serialports[sp_idx];

    ssize_t bytes_written = write(sp.fd, &byte, 1);

    if (bytes_written < 0) {
        printf(ERROR "Faild to write to the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
        return false;
    }

    if (bytes_written == 1) return true;
    return false;
}
    
SPAPI uint64_t serialport_write_bytes(uint32_t sp_idx, void* bytes, uint64_t bytes_size)
{
    if (!valid_idx_and_connected(sp_idx)) return false;

    Serialport& sp = gsps.serialports[sp_idx];

    ssize_t bytes_written = write(sp.fd, bytes, bytes_size);

    if (bytes_written < 0) {
        printf(ERROR "Faild to write to the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
        return 0;
    }

    return bytes_written;
}

// SPAPI bool serialport_read_object(SP_IDX sp_idx, void* object_buffer, uint64_t object_size)
// {
//     if (object_size > SERIALPORTLIB_OBJECT_MAX_BYTES) {
//         printf(ERROR "Your object of size '%lu' is larger than the maximum of '%d'\n", object_size, SERIALPORTLIB_OBJECT_MAX_BYTES);
//         return false;
//     }
    
//     if (!valid_idx_and_connected(sp_idx)) return false;
    
//     Serialport& sp = gsps.serialports[sp_idx];

//     if (sp.delimiter_length == 0) {
//         printf(ERROR "A delimiter is required to read objects. Please set the delimiter for the serialport index '%u'.\n", sp_idx);
//         return false;
//     }

//     // try to get enough bytes to return a single object
//     int bytes_required = object_size + sp.delimiter_length - sp.leftovers_length;
    
//     uint8_t buffer[SERIALPORTLIB_OBJECT_MAX_BYTES];
//     ssize_t bytes_read = read(sp.fd, buffer, bytes_required);

//     if (bytes_read < 0) {
//         printf(ERROR "Faild to read bytes from the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
//         return false;
//     }

//     if (bytes_read > 0) {
//         memcpy(&sp.leftover_bytes[sp.leftovers_length], buffer, bytes_read);
//         sp.leftovers_length += bytes_read;
    
//         if (bytes_read < bytes_required) {
//             return false;
//         }
//         // compare the last 'delimiter_length' bytes with the delimiter
//         else if (memcmp(&sp.leftover_bytes[object_size], sp.delimiter, sp.delimiter_length) == 0) {
//             memcpy(object_buffer, sp.leftover_bytes, object_size);
//             sp.leftovers_length = 0;
//             return true;
//         }
//         else {
//             printf(ERROR "Didn't find the delimiter at the expected position in the serial data stream. "
//                    "Advancing one byte at a time until we find it.\n");

//             int skipped_bytes = 0;
            
//             while (true)
//             {
//                 for (uint64_t i = 1; i < object_size + sp.delimiter_length; ++i) {
//                     sp.leftover_bytes[i - 1] = sp.leftover_bytes[i]; // shift the 'leftovers' by one byte
//                 }
//                 sp.leftovers_length--;
//                 skipped_bytes++;
            
//                 // get just the next byte
//                 uint8_t next_byte;
//                 ssize_t bytes_read = read(sp.fd, &next_byte, 1);

//                 if (bytes_read < 0) {
//                     printf(ERROR "Faild to read bytes from the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
//                     return false;
//                 }

//                 if (bytes_read == 1) {
//                     sp.leftover_bytes[sp.leftovers_length] = next_byte;
//                     sp.leftovers_length++;
    
//                     // compare the last 'delimiter_length' bytes with the delimiter
//                     if (memcmp(&sp.leftover_bytes[object_size], sp.delimiter, sp.delimiter_length) == 0) {
//                         memcpy(object_buffer, sp.leftover_bytes, object_size);
//                         sp.leftovers_length = 0;
//                         printf(WARNING "Found the delimiter and skipped '%d' bytes.\n", skipped_bytes);
//                         // Now that we are aligned with the stream again, we can try to actually get the next object
//                         return serialport_read_object(sp_idx, object_buffer, object_size);
//                     }
//                     else {
//                         continue;
//                     }
//                 }                
//                 return false;
//             }
//         }
//     }
    
//     return false;
// }
