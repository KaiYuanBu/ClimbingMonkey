import canopen

error_code = 0x05040000

# Check if error_code is equal to 0x05040000
# print(canopen.SdoAbortedError.CODES)

# # if error_code == canopen.SdoAbortedError.CODES:
# #     print("Error code is 0x05040000 (ERROR_DEVICE_STATE)")
# # else:
# #     print("Error code is not 0x05040000")

# Get the integer part of the error code
error_code_int = error_code & 0xFFFFF000

# Check if error_code_int matches any codes in canopen.SdoAbortedError.CODES
if error_code_int in canopen.SdoAbortedError.CODES:
    print("Error code {} matches with an SDO aborted error code.".format(hex(error_code)))
    # You can also print the description of the error code
    print("Description:", canopen.SdoAbortedError.CODES[error_code_int])
else:
    print("Error code {} does not match any known SDO aborted error code.".format(hex(error_code)))