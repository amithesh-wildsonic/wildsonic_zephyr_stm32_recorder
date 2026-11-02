import argparse
from datetime import datetime
import ntplib
import re
import serial
import time


def collect_ntp_time():
    ''' Collect the NTP time '''
    c = ntplib.NTPClient()
    response = c.request('pool.ntp.org', version=3)
    return datetime.fromtimestamp(response.tx_time)


def wait_for_prompt(ser, prompt=b'>', verbose=False):
    """
    Wait for the prompt after sending a command.
    Args:
        ser (serial.Serial): The serial connection to the device.
        prompt (bytes): The prompt to wait for. Default is b'>'.
        verbose (bool): If True, prints the response received every 3 seconds. Default is False.
    Returns:
        tuple:
            - bytes: Response received
            - datetime.timedelta: Time taken to receive the response
    """
    # Wait for prompt to appear.
    # Every 3 seconds, print the response received so far.

    start_time = datetime.now()
    last_time = start_time
    last_print_offset = 0

    response = b''
    while response.find(prompt) == -1:
        response += ser.read(1)

        if verbose:
            if (datetime.now() - last_time).total_seconds() > 3:
                print(response[last_print_offset:], end='')
                last_time = datetime.now()

    if verbose:
        if last_print_offset < len(response):
            print(response[last_print_offset:], end='')

    return response, datetime.now() - start_time


def send_command(ser, command, verbose=False):
    """
    Send a command to the device.
    Args:
        ser (serial.Serial): The serial connection to the device.
        command (str): Command to send.
        verbose (bool, optional): Print the response received. Defaults to False.
    Returns:
        tuple:
            - bytes: Response received
            - datetime.timedelta: Time taken to receive the response
    """

    if len(command) > 0:
        print('Sending command: ' + command)
    ser.write(command.encode() + b'\r')
    return wait_for_prompt(ser, verbose=verbose)


def noop_command(ser):
    """
    Send a newline command to the device.

    Parameters:
        ser (serial.Serial): The serial connection to the device.

    Returns:
        tuple:
            - bytes: Response received
            - datetime.timedelta: Time taken to receive the response
    """
    return send_command(ser, '')


def reboot_command(ser):
    """
    Send a reboot command to the device.

    Args:
        ser (serial.Serial): The serial connection to the device.

    Returns:
        tuple:
            - bytes: Response received during the reboot
            - datetime.timedelta: Time to RTC Initialization
            - datetime.timedelta: Time to Last System Init Done
    """
    send_command(ser, 'ARBT')

    response_1, reboot_time_1 = wait_for_prompt(ser, prompt=b'RTC Time UTC', verbose=False)
    response_2, reboot_time_2 = wait_for_prompt(ser, prompt=b'Last System Init Done', verbose=False)

    # Return time to RTC Initialization and time to Last System Init Done
    # in addition to the response received during the reboot
    return response_1 + response_2, reboot_time_1, reboot_time_1 + reboot_time_2,


def set_datetime(ser, datetime):
    """
    Set the datetime on the device.

    Args:
        ser (serial.Serial): The serial connection to the device.
        datetime (datetime): The datetime to set on the device.
    """
    response, _ = send_command(ser, 'DATETIME ' + datetime.strftime('%Y/%m/%d %H:%M:%S %z'))

    # If the current time is ahead of the programmed time, the device will not accept the time.
    # In this case, we flag it as an error.
    match = re.search(r'Current time Local is (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3} [+-]\d{2}:\d{2})', response.decode())
    if match:
        current_time = datetime.strptime(match.group(1), '%Y-%m-%d %H:%M:%S.%f %z')
        if current_time > datetime:
            raise ValueError('Current time is ahead of the programmed time. Please wait for the current time to be less than the programmed time.')


def get_datetime(ser):
    """
    Get the current datetime from the device.

    Args:
        ser (serial.Serial): The serial connection to the device.

    Returns:
        datetime: The current datetime of the device.
    """
    response, _ = send_command(ser, 'DATETIME')

    # Extract the datetime from the response
    # Response should be of the format: Current time Local is YYYY-MM-DD HH:MM:SS.mmm TZ
    match = re.search(r'Current time Local is (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3} [+-]\d{2}:\d{2})', response.decode())
    if match:
        return datetime.strptime(match.group(1), '%Y-%m-%d %H:%M:%S.%f %z')
    else:
        return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Set the datetime of the device.')
    parser.add_argument('-p', '--port', help='Serial port. If not provided, it will use the first available port.')
    parser.add_argument('-d', '--datetime', help='Datetime in the format YYYY/MM/DD HH:MM:SS [+-]ZZZZ. If the time is not provided, it will use the current network protocol time.')
    parser.add_argument('-b', '--baudrate', help='Baud rate', default=115200)

    args = parser.parse_args()

    if not args.port:
        # Search for the first available port
        print('Searching for available port...')
        for i in range(256):
            try:
                ser = serial.Serial('COM' + str(i), args.baudrate, timeout=1)
                ser.close()
                args.port = 'COM' + str(i)
                break
            except:
                pass

        if not args.port:
            print('No available port found.')
            exit(1)

    print('Using port ' + args.port)

    if args.datetime:
        try:
            date_time = datetime.strptime(args.datetime, '%Y/%m/%d %H:%M:%S %z')
        except ValueError:
            # Try without the timezone
            try:
                date_time = datetime.strptime(args.datetime, '%Y/%m/%d %H:%M:%S')
                # Add the current timezone
                date_time = date_time.astimezone()
            except ValueError:
                print('Invalid datetime format. Use YYYY/MM/DD HH:MM:SS [+-]ZZ:ZZ')
                exit(1)


    ser = serial.Serial(args.port, args.baudrate, timeout=1)
    noop_command(ser)

    if not args.datetime:
        print('Measuring time to reboot so we can set the time as accurately as possible...')
        _, first_measured_reboot_time_to_rtc_init, _ = reboot_command(ser)
        print('Measured time from reboot command to rtc initialization: ' + str(first_measured_reboot_time_to_rtc_init))

        date_time = collect_ntp_time()
        # date_time = datetime.now()
        date_time = date_time.astimezone()
        date_time += first_measured_reboot_time_to_rtc_init

    set_datetime(ser, date_time)
    _, second_measured_reboot_time_to_rtc_init, _ = reboot_command(ser)
    print('2nd reboot command to rtc initialization: ' + str(second_measured_reboot_time_to_rtc_init))

    if first_measured_reboot_time_to_rtc_init > second_measured_reboot_time_to_rtc_init:
        diff = first_measured_reboot_time_to_rtc_init - second_measured_reboot_time_to_rtc_init
    else:
        diff = second_measured_reboot_time_to_rtc_init - first_measured_reboot_time_to_rtc_init
    print('Time setting might be off by ' + str(diff))

    # Sleep for 2 seconds just to make sure everything is clear on the console
    time.sleep(2)

    programmed_datetime = get_datetime(ser)
    print('Datetime currently is ' + programmed_datetime.strftime('%Y/%m/%d %H:%M:%S %z'))

    ser.close()

