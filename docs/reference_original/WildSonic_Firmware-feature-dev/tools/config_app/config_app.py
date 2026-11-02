import tkinter as tk
from tkinter import ttk, filedialog
import configparser
import functools
import re
from datetime import datetime, timedelta
import os
import sys
import time
from tkinter import messagebox

LOWER_CUTOFF_FREQ_INI_FIELD = "lower_cutoff_frequency"
UPPER_CUTOFF_FREQ_INI_FIELD = "upper_cutoff_frequency"
DATE_RANGE_INI_FIELD = "date_range"
TIME_OF_DAY_INI_FIELD = "time_of_day"
APP_VERSION = "1.1.1"

def enable_disable_time_range(comboboxes, enable_var):
    """
    Enable or disable the comboboxes based on the enable_var value.

    Args:
        comboboxes (list): A list of combobox widgets to be enabled or disabled.
        enable_var (tkinter.Variable): A Tkinter variable that determines whether to enable or disable the comboboxes.
    """
    for combobox in comboboxes:
        if enable_var.get():
            combobox.configure(state="readonly")
        else:
            combobox.configure(state="disabled")

class ConfigGeneratorApp:
    """
    A class to generate configuration INI files for the WildSonic Audio Recorder.
    Attributes:
        master (tk.Tk): The main window of the application.
        config_params (list): A list of dictionaries containing configuration parameters.
    Methods:
        __init__(self, master):
            Initializes the ConfigGeneratorApp with the given master window.
        show_tooltip(self, event, hover_text):
            Displays a tooltip with the given hover text near the cursor.
        hide_tooltip(self, event):
            Hides the currently displayed tooltip.
        show_about_popup(self):
            Displays an "About" popup with information about the application.
        create_widgets(self):
            Creates and arranges the widgets in the application window.
        date_range_text_box_to_ini_value(self, ini_field):
            Converts the date range text box values to a string suitable for the INI file.
        multi_time_range_text_box_to_ini_value(self, ini_field):
            Converts the multi-time range text box values to a list of strings suitable for the INI file.
        save_config(self):
            Saves the current configuration to an INI file.
    """
    def __init__(self, master):
        self.master = master
        self.master.title("WildSonic Configuration INI File Generator " + APP_VERSION)
        script_dir = os.path.dirname(__file__)

        platform = sys.platform.lower()
        if platform.startswith('win'):
            icon_path = os.path.join(script_dir, r"favicon.ico")

            try:
                from ctypes import windll  # Only exists on Windows.

                myappid = "wildly.wildsonic.config_app." + APP_VERSION
                windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
            except ImportError:
                pass
        else:
            icon_path = None

        if icon_path is not None:
            self.master.iconbitmap(icon_path)

        self.VALID_SAMPLE_RATES = [16000, 22050, 24000, 32000, 44100, 48000]

        self.config_params = [
            # Tab index, Display name, hover text
            # same row, INI field name, menu type, max per row
            # Is number, Is Integer, values, default value
            {"tab_index": 0, "display_name": "Sampling Rate", "hover_text": "In Hz",
             "same_row": False, "ini_field": "sampling_frequency", "menu_type": "drop-down", "max_per_row": 3,
             "is_number": True, "is_integer": True, "values": self.VALID_SAMPLE_RATES, "default": 48000},
            {"tab_index": 0, "display_name": "Lower Frequency", "hover_text": "Lower Cutoff Filter\nFrequency in Hz",
             "same_row": False, "ini_field": LOWER_CUTOFF_FREQ_INI_FIELD, "menu_type": "text-box", "max_per_row": 3,
             "is_number": True, "is_integer": True, "values": (0, None), "default": 0},
            {"tab_index": 0, "display_name": "Upper Frequency", "hover_text": "Upper Cutoff Filter\nFrequency in Hz.\nMaximum value is half\nof the sampling rate.",
             "same_row": False, "ini_field": UPPER_CUTOFF_FREQ_INI_FIELD, "menu_type": "text-box", "max_per_row": 3,
             "is_number": True, "is_integer": True, "values": (0, None), "default": self.VALID_SAMPLE_RATES[-1]//2},
            {"tab_index": 0, "display_name": "Number of Channels\n", "hover_text": None,
             "same_row": False, "ini_field": "number_of_channels", "menu_type": "drop-down", "max_per_row": 3,
             "is_number": True, "is_integer": True, "values": [1], "default": 1},
            {"tab_index": 0, "display_name": "Bits per Sample\n", "hover_text": None,
             "same_row": False, "ini_field": "bits_per_sample", "menu_type": "drop-down", "max_per_row": 3,
             "is_number": True, "is_integer": True, "values": [16, 32], "default": 16},
            {"tab_index": 0, "display_name": "Mic Gain", "hover_text": "Microphone Gain\nin dB",
             "same_row": False, "ini_field": "mic_gain", "menu_type": "slider", "max_per_row": 3,
             "is_number": True, "is_integer": False, "values": (-24, 24), "default": 0},
            {"tab_index": 0, "display_name": "Time per Audio File", "hover_text": "Audio duration in seconds of each audio file",
             "same_row": False, "ini_field": "record_length", "menu_type": "text-box", "max_per_row": 3,
             "is_number": True, "is_integer": True, "values": (0, 86400), "default": 600},
            {"tab_index": 0, "display_name": "Sleep Time", "hover_text": "Idle non-recording time in seconds\nbetween each audio file recording.\nSet to 0 for continuous recording.",
             "same_row": False, "ini_field": "sleep_interval", "menu_type": "text-box", "max_per_row": 3,
             "is_number": True, "is_integer": True, "values": (0, 86400), "default": 0},
            {"tab_index": 0, "display_name": "Folder Structure", "hover_text": "Indicates how the audio files\nshould be organized on the SD card.\nflat - top directory\ndaily - one folder for each day\nhourly - one folder for each hour",
             "same_row": False, "ini_field": "folder_structure", "menu_type": "drop-down", "max_per_row": 3,
             "is_number": False, "is_integer": False, "values": ["flat", "daily", "hourly"], "default": "hourly"},
            {"tab_index": 1, "display_name": "Date Range", "hover_text": "Leave unset if everyday recording",
             "same_row": True, "ini_field": DATE_RANGE_INI_FIELD, "menu_type": "date-range-text-box", "max_per_row": 1,
             "is_number": False, "is_integer": False, "values": None, "default": None},
            {"tab_index": 1, "display_name": "Days of Week", "hover_text": "All days should be checked if\nrecording every day of the week",
             "same_row": True, "ini_field": "days_of_week", "menu_type": "day-of-week-checkboxes", "max_per_row": 1,
             "is_number": False, "is_integer": False, "values": ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"], "default": [1, 1, 1, 1, 1, 1, 1]},
            {"tab_index": 1, "display_name": "Time Ranges", "hover_text": "24 hour time format.\nLeave unset if all-day recording",
             "same_row": True, "ini_field": TIME_OF_DAY_INI_FIELD, "menu_type": "multi-time-range-text-box", "max_per_row": 1,
             "is_number": False, "is_integer": False, "values": None, "default": None},
            {"tab_index": 0, "display_name": "System Time Offset", "hover_text": "System Time Offset in seconds.\nClock on recorder will be set to\ncurrent time now plus this value.\nOnly set to non-zero if time\non recorder is not accurate.",
             "same_row": False, "ini_field": "system_time_offset", "menu_type": "text-box", "max_per_row": 1,
             "is_number": True, "is_integer": True, "values": (0, 100000), "default": 0},
            # {"tab_index": 2, "display_name": "Transmission\nMode", "hover_text": "Only applicable for devices with connectivity.",
            #  "same_row": False, "ini_field": "transmission_mode", "menu_type": "drop-down", "max_per_row": 2,
            #  "is_number": False, "is_integer": False, "values": ["2G", "3G", "4G"], "default": "4G"},
            # {"tab_index": 2, "display_name": "SMS Status Frequency", "hover_text": "Indicates how often in seconds\nthe device should report status to server.\nOnly applicable for devices with connectivity.",
            #  "same_row": False, "ini_field": "sms_frequency", "menu_type": "slider", "max_per_row": 2,
            #  "is_number": True, "is_integer": True, "values": (300, 86400), "default": 3600},
        ]

        self.create_widgets()

    def show_tooltip(self, event, hover_text):
        """
        Display a tooltip window near the cursor with the provided hover text.
        Args:
            event (tkinter.Event): The event object containing information about the mouse event.
            hover_text (str): The text to be displayed in the tooltip.
        Creates:
            A tooltip window that appears near the cursor and displays the provided hover text.
            The tooltip window is destroyed when the mouse leaves the tooltip label.
        """

        self.tooltip_text = hover_text

        # Create a tooltip window as an instance variable
        self.tooltip_window = tk.Toplevel(self.master)
        self.tooltip_window.wm_overrideredirect(True)  # Remove window decorations
        self.tooltip_window.wm_geometry(f"+{event.x_root + 10}+{event.y_root + 10}")  # Position the window near the cursor

        # Create a label to display the tooltip text
        self.tooltip_label = tk.Label(self.tooltip_window, text=hover_text, background="#ffffe0", relief=tk.SOLID, borderwidth=1, padx=5, pady=2)
        self.tooltip_label.pack()

        # Bind the tooltip window to the leave event of the label
        self.tooltip_label.bind("<Leave>", lambda event: self.tooltip_window.destroy())

        # Set the focus to the tooltip window
        self.tooltip_window.focus_set()

    def hide_tooltip(self, event):
        """
        Hides the tooltip by destroying the tooltip window if it exists.
        Parameters:
        event (Event): The event that triggered the hiding of the tooltip.
        """

        self.tooltip_text = None

        if self.tooltip_window is not None:
            self.tooltip_window.destroy()
            self.tooltip_window = None
            self.tooltip_label = None

    def show_about_popup(self):
        """
        Displays an 'About' popup with information about the WildSonic Config Generator App.

        The popup includes:
        - Company name
        - Application name
        - Application version
        - Description of the app's purpose
        - Information about the configuration file generated by the app
        - Credit for in-app icons

        The configuration file is saved as 'config.ini' in the selected directory and is used to set the parameters of the WildSonic Audio Recorder.
        """
        about_text = "Wildly Tech Pvt. Ltd.\n"
        about_text += "WildSonic Config Generator App.\n"
        about_text += "Version " + APP_VERSION + "\n\n"
        about_text += "This app generates a configuration file for the WildSonic Audio Recorder.\n\n"
        about_text += "The configuration file is saved as 'config.ini' in the selected directory.\n\n"
        about_text += "The configuration file is used to set the parameters of the WildSonic Audio Recorder.\n\n"
        about_text += "In-app icons from icons8.com\n\n"
        messagebox.showinfo("About", about_text)

    def create_widgets(self):
        """
        Create and configure the widgets for the application.
        This method sets up a tabbed interface with multiple tabs, and populates each tab with various widgets
        based on the configuration parameters provided in `self.config_params`. The widgets can include labels,
        drop-down menus, sliders, text boxes, date range selectors, multi-time range selectors, day-of-week checkboxes,
        and regular checkboxes. Each widget is configured according to its respective settings.
        The method also adds a "Save Config" button and an "About" button at the bottom of the interface.
        Attributes:
            MAX_ROW_LIMIT (int): The maximum number of rows allowed per tab.
            row_counter (int): Counter to keep track of the current row.
            row_limit (int): The limit of rows for the current tab.
            notebook (ttk.Notebook): The notebook widget containing the tabs.
            tab_names (list): List of tab names.
            tab_frames (list): List of frames corresponding to each tab.
            row_frame (tk.Frame): Frame for organizing widgets in rows.
            item_frame (tk.Frame): Frame for individual items within a row.
            label_frame (tk.Frame): Frame for labels and optional question icons.
            menu (various): The widget created based on the menu type (e.g., drop-down, slider, text-box, etc.).
        Note:
            The method assumes that `self.config_params` is a list of dictionaries, where each dictionary contains
            the configuration for a single widget. The keys in each dictionary should include:
            - "tab_index": Index of the tab where the widget should be placed.
            - "display_name": The display name of the widget.
            - "hover_text": Tooltip text for the widget (optional).
            - "same_row": Boolean indicating if the widget should be placed on the same row as the previous widget.
            - "ini_field": The attribute name to store the widget reference.
            - "menu_type": The type of widget to create (e.g., "drop-down", "slider", "text-box", etc.).
            - "max_per_row": Maximum number of widgets per row.
            - "values": List of values for the widget (e.g., options for drop-down menus).
            - "default": Default value for the widget (optional).
        """

        MAX_ROW_LIMIT = 100000

        row_counter = 0
        row_limit = MAX_ROW_LIMIT

        # Create a notebook (tabbed interface) within master
        notebook = ttk.Notebook(self.master)
        notebook.pack(side="top", fill=tk.BOTH, expand=True)

        tab_names = ["General", "Scheduling"]
        tab_frames = []

        for tab_name in tab_names:
            tab_frame = tk.Frame(notebook)
            tab_frame.pack(side="left", fill=tk.BOTH, expand=True)
            notebook.add(tab_frame, text=tab_name)

            tab_frames.append(tab_frame)

        for entry in self.config_params:

            tab_index = entry["tab_index"]
            param_name = entry["display_name"]
            hover_text = entry["hover_text"]
            same_row = entry["same_row"]
            ini_field = entry["ini_field"]
            menu_type = entry["menu_type"]
            max_row_count = entry["max_per_row"]
            values = entry["values"]
            default = entry["default"]

            tab_index_frame = tab_frames[tab_index]

            row_limit = min(row_limit, max_row_count)

            newrow = False
            position_args = {}
            add_new_row_frame = True
            if row_limit == 1:
                row_counter = 0
                row_limit = MAX_ROW_LIMIT
            elif row_counter >= row_limit:
                row_counter = 1
                row_limit = MAX_ROW_LIMIT
            else:
                if row_counter != 0:
                    add_new_row_frame = False
                position_args = {"side":tk.LEFT}
                row_counter += 1
                if row_counter == row_limit:
                    row_counter = 0
                    row_limit = MAX_ROW_LIMIT

            if add_new_row_frame:
                row_frame = tk.Frame(tab_index_frame)  # Create a new frame inside tab_index_frame
                row_frame.pack(pady=2)  # Pack the new frame into tab_index_frame

            item_frame = tk.Frame(row_frame)
            item_frame.pack(padx=5, pady=2, **position_args)

            # if question icon is needed, then create a frame to hold the question icon and the label
            label_frame = tk.Frame(item_frame)
            label = tk.Label(label_frame, text=param_name)
            label.pack(side=tk.LEFT)

            if same_row:
                label_frame.pack(side=tk.LEFT, padx=5)
            else:
                label_frame.pack(pady=2)

            if hover_text is not None:
                script_dir = os.path.dirname(__file__)
                icon_path = os.path.join(script_dir, r"icons8-help-32.png")

                question_icon = tk.PhotoImage(file=icon_path).subsample(2)
                question_button = tk.Button(label_frame, image=question_icon, bd=0, relief=tk.FLAT, cursor="question_arrow")
                question_button.image = question_icon
                question_button.pack(side=tk.LEFT)
                question_button.bind("<Enter>", lambda event, text=hover_text: self.show_tooltip(event, text))
                question_button.bind("<Leave>", lambda event: self.hide_tooltip(event))

            if same_row:
                # Add colon to separate label and menu
                colon_label = tk.Label(label_frame, text=":")
                colon_label.pack(side=tk.LEFT)

            if menu_type == "drop-down":
                menu = ttk.Combobox(item_frame, values=values, state="readonly", width=8)
                menu.pack(pady=2)
                if default is not None:
                    menu.set(default)
            elif menu_type == "slider":
                menu = tk.Scale(item_frame, from_=values[0], to=values[1], orient=tk.HORIZONTAL)
                menu.pack(pady=2)
                if default is not None:
                    menu.set(default)
            elif menu_type == "text-box":
                menu = tk.Entry(item_frame, width=8)
                menu.pack(pady=2)
                if default is not None:
                    menu.insert(0, str(default))  # Set default value
            elif menu_type == "date-range-text-box":
                # 6 dropdowns for date range (2 for year, 2 for month, 2 for day)
                frame = tk.Frame(item_frame)
                frame.pack(pady=2)
                dropdowns = []

                # Add year dropdown (values go from 2025-2099)
                # Add month dropdown (values go from 01-12)
                # Add day dropdown (values go from 01-31)
                year_values = [str(year) for year in range(2025, 2100)]
                year_values.insert(0, "YYYY")  # Allow empty value
                month_values = [f"{month:02d}" for month in range(1, 13)]
                month_values.insert(0, "MM")  # Allow empty value
                day_values = [f"{day:02d}" for day in range(1, 32)]
                day_values.insert(0, "DD")  # Allow empty value

                for i in range(2):
                    dropdown = ttk.Combobox(frame, values=year_values, state="readonly", width=6)
                    dropdown.set("YYYY")
                    dropdown.pack(side=tk.LEFT)
                    dropdowns.append(dropdown)

                    label = tk.Label(frame, text="/")
                    label.pack(side=tk.LEFT)

                    dropdown = ttk.Combobox(frame, values=month_values, state="readonly", width=4)
                    dropdown.set("MM")
                    dropdown.pack(side=tk.LEFT)
                    dropdowns.append(dropdown)

                    label = tk.Label(frame, text="/")
                    label.pack(side=tk.LEFT)

                    dropdown = ttk.Combobox(frame, values=day_values, state="readonly", width=4)
                    dropdown.set("DD")
                    dropdown.pack(side=tk.LEFT)
                    dropdowns.append(dropdown)

                    if i == 0:
                        label = tk.Label(frame, text="-")
                        label.pack(side=tk.LEFT)

                menu = dropdowns
                if default is not None:
                    menu.set(default)
            elif menu_type == "multi-time-range-text-box":
                multirow_item_frame = tk.Frame(item_frame)
                multirow_item_frame.pack(pady=2)
                rows = []

                # Up to 10 sets of dropdown menus for time range (2 for hour, 2 for minute, 2 for second)
                for row_index in range(10):
                    frame = tk.Frame(multirow_item_frame)
                    frame.pack(pady=2)

                    comboboxes = []

                    # Add enable checkbox
                    enable_var = tk.IntVar()
                    enable_checkbox = tk.Checkbutton(frame, text="",
                                                     variable=enable_var,
                                                     command=lambda cb=comboboxes, en=enable_var: enable_disable_time_range(cb, en))
                    enable_checkbox.pack(side=tk.LEFT)

                    hour_values = [f"{hour:02d}" for hour in range(0, 24)]
                    hour_values.insert(0, "HH")  # Allow empty value
                    minute_values = [f"{minute:02d}" for minute in range(0, 60)]
                    minute_values.insert(0, "MM")  # Allow empty value
                    second_values = [f"{second:02d}" for second in range(0, 60)]
                    second_values.insert(0, "SS")  # Allow empty value

                    for i in range(2):
                        dropdown = ttk.Combobox(frame, values=hour_values, state="disabled", width=4)
                        dropdown.set("HH")
                        dropdown.pack(side=tk.LEFT)
                        comboboxes.append(dropdown)

                        label = tk.Label(frame, text=":")
                        label.pack(side=tk.LEFT)

                        dropdown = ttk.Combobox(frame, values=minute_values, state="disabled", width=4)
                        dropdown.set("MM")
                        dropdown.pack(side=tk.LEFT)
                        comboboxes.append(dropdown)

                        label = tk.Label(frame, text=":")
                        label.pack(side=tk.LEFT)

                        dropdown = ttk.Combobox(frame, values=second_values, state="disabled", width=4)
                        dropdown.set("SS")
                        dropdown.pack(side=tk.LEFT)
                        comboboxes.append(dropdown)

                        if i == 0:
                            label = tk.Label(frame, text="-")
                            label.pack(side=tk.LEFT)

                    rows.append((enable_var, comboboxes))

                menu = rows
                if default is not None:
                    menu.set(default)
            elif menu_type == "day-of-week-checkboxes":
                # Of the form: "Mon,Tue,Wed,Thu,Fri,Sat,Sun"
                frame = tk.Frame(item_frame)
                frame.pack(pady=2)
                checkboxes = [tk.IntVar() for _ in range(7)]

                checkbox_row_1 = tk.Frame(frame)
                checkbox_row_1.pack()
                for i, day in enumerate(["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"]):
                    checkbox = tk.Checkbutton(checkbox_row_1, text=day, variable=checkboxes[i])
                    checkbox.pack(side=tk.LEFT)

                menu = checkboxes
                if default is not None:
                    for i, day in enumerate(default):
                        checkboxes[i].set(int(day))
            elif menu_type == "checkboxes":
                frame = tk.Frame(item_frame)
                frame.pack(pady=2)
                checkboxes = [tk.IntVar() for _ in values]
                for i, value in enumerate(values):
                    checkbox = tk.Checkbutton(frame, text=value, variable=checkboxes[i])
                    checkbox.pack(side=tk.LEFT)
                menu = checkboxes
                menu.set(default)

            setattr(self, ini_field, menu)

        last_row = tk.Frame(self.master)
        save_button = tk.Button(last_row, text="Save Config", command=self.save_config)
        save_button.pack(side=tk.TOP, pady=2)

        about_button = tk.Button(last_row,
                                 text="About",
                                 command=self.show_about_popup,
                                 relief=tk.FLAT,
                                 fg="blue",
                                 font=("Arial", 8, "underline"))
        about_button.pack(side=tk.TOP)

        last_row.pack(side=tk.BOTTOM)

    def date_range_text_box_to_ini_value(self, field_date_dropdowns):
        """
        Converts the date range from text boxes to an INI file compatible value.
        This function validates the start and end dates provided in the text boxes.
        If the dates are invalid or the start date is greater than the end date,
        it sets the dates to an invalid value ("9999/99/99").
        Args:
            field_date_dropdowns (list): A list of 6 dropdown objects representing the date range.
        Returns:
            str: A string representing the date range in the format "YYYY/MM/DD-YYYY/MM/DD".
                 If the dates are invalid, the string will include "9999/99/99" to indicate no date selected.
        """

        INVALID_YEAR = "9999"
        INVALID_MONTH = "99"
        INVALID_DAY = "99"

        def set_date_to_invalid_value_in_range(date_range, index):
            '''
            :param date_range: list of date values
            :param index 0 for start date, 1 for end date
            :return: None
            '''
            if index == 0:
                date_range[index*3] = INVALID_YEAR
                date_range[index*3+1] = INVALID_MONTH
                date_range[index*3+2] = INVALID_DAY
            else:
                date_range[index*3] = INVALID_YEAR
                date_range[index*3+1] = INVALID_MONTH
                date_range[index*3+2] = INVALID_DAY

        value = [dropdown.get() for dropdown in field_date_dropdowns]

        start_date = value[:3]
        end_date = value[3:]

        # Validations for date format:
        # 1. Year should be 4 digits
        # 2. Month should be 2 digits
        # 3. Day should be 2 digits
        # 4. Start date should be less than or equal to end date
        # 5. Date should be on the calendar (e.g. 2021-02-29 is invalid)
        start_date_valid = False
        end_date_valid = False

        if re.match(r"\d{4}-\d{2}-\d{2}", '-'.join(start_date)):
            # Check if the date is valid
            try:
                datetime.strptime('-'.join(start_date), "%Y-%m-%d")
                start_date_valid = True
            except ValueError:
                pass

        if re.match(r"\d{4}-\d{2}-\d{2}", '-'.join(end_date)):
            # Check if the date is valid
            try:
                datetime.strptime('-'.join(end_date), "%Y-%m-%d")
                end_date_valid = True
            except ValueError:
                pass

        if start_date_valid and end_date_valid:
            if '-'.join(start_date) > '-'.join(end_date):
                # If start date is greater than end date, then both are invalid
                start_date_valid = False
                end_date_valid = False

        if not start_date_valid:
            set_date_to_invalid_value_in_range(value, 0)

        if not end_date_valid:
            set_date_to_invalid_value_in_range(value, 1)

        value = "-".join(["/".join(value[i:i+3]) for i in range(0, len(value), 3)])
        if not start_date_valid or not end_date_valid:
            value += f" # {INVALID_YEAR}/{INVALID_MONTH}/{INVALID_DAY} value indicates no date selected"

        return value

    def multi_time_range_text_box_to_ini_value(self, field_time_range):
        """
        Converts multiple time range selections from a text box to INI format values.
        This function processes a list of time ranges, validates them, and converts them
        into a specific INI format. Invalid time ranges are marked with a specific invalid
        time range string.
        Args:
            field_time_range (list): A list of tuples where each tuple contains:
                - enable_var (tkinter.BooleanVar): A variable indicating if the time range is enabled.
                - comboboxes (list): A list of 6 tkinter Combobox widgets representing the time range.
        Returns:
            list: A list of strings representing the time ranges in INI format. Invalid time ranges
                  are marked with a specific invalid time range string.
        """

        values = []

        INVALID_START_HOUR = "00"
        INVALID_START_MINUTE = "00"
        INVALID_START_SECOND = "00"

        INVALID_END_HOUR = "24"
        INVALID_END_MINUTE = "00"
        INVALID_END_SECOND = "00"

        INVALID_START_TIME = f"{INVALID_START_HOUR}:{INVALID_START_MINUTE}:{INVALID_START_SECOND}"
        INVALID_END_TIME = f"{INVALID_END_HOUR}:{INVALID_END_MINUTE}:{INVALID_END_SECOND}"

        INVALID_TIME_RANGE = f"{INVALID_START_TIME}-{INVALID_END_TIME}"
        INVALID_TIME_RANGE_STRING = INVALID_TIME_RANGE + " # 00:00:00-24:00:00 value indicates invalid time range"

        def set_time_to_invalid_value_in_range(time_range, index):
            '''
            :param time_range: list of time values
            :param index 0 for start time, 1 for end time
            :return: None
            '''
            if index == 0:
                time_range[index*3] = INVALID_START_HOUR
                time_range[index*3+1] = INVALID_START_MINUTE
                time_range[index*3+2] = INVALID_START_SECOND
            else:
                time_range[index*3] = INVALID_END_HOUR
                time_range[index*3+1] = INVALID_END_MINUTE
                time_range[index*3+2] = INVALID_END_SECOND

        for time_range_index, (enable_var, comboboxes) in enumerate(field_time_range):

            time_range = [dropdown.get() for dropdown in comboboxes]

            # Validate time format
            if not enable_var.get():
                valid_start_time = False
                valid_end_time = False
            else:
                start_time = time_range[:3]
                end_time = time_range[3:]

                # Validations for time format:
                # 1. Hour should be 2 digits
                # 2. Minute should be 2 digits
                # 3. Second should be 2 digits
                # 4. Start time should be less than end time
                valid_start_time = re.match(r"\d{2}:\d{2}:\d{2}", ':'.join(start_time))
                valid_end_time = re.match(r"\d{2}:\d{2}:\d{2}", ':'.join(end_time))

                if valid_start_time and valid_end_time:
                    if ':'.join(start_time) >= ':'.join(end_time):
                        # If start time is greater than end time, then both are invalid
                        valid_start_time = False
                        valid_end_time = False

            if not valid_start_time:
                set_time_to_invalid_value_in_range(time_range, 0)

            if not valid_end_time:
                set_time_to_invalid_value_in_range(time_range, 1)

            time_range = "-".join([":".join(time_range[i:i+3]) for i in range(0, len(time_range), 3)])

            values.append(time_range)

        for i in range(len(values)):
            for j in range(i+1, len(values)):
                valid_i = values[i] != INVALID_TIME_RANGE
                valid_j = values[j] != INVALID_TIME_RANGE

                if valid_i and valid_j:
                    start_time_i, end_time_i = values[i].split("-")
                    start_time_j, end_time_j = values[j].split("-")

                    start_time_i = datetime.strptime(start_time_i, "%H:%M:%S")

                    one_day_later = datetime.strptime("00:00:00", "%H:%M:%S") + timedelta(days=1)

                    if end_time_i == INVALID_END_TIME:
                        end_time_i = one_day_later
                    else:
                        end_time_i = datetime.strptime(end_time_i, "%H:%M:%S")

                    start_time_j = datetime.strptime(start_time_j, "%H:%M:%S")

                    if end_time_j == INVALID_END_TIME:
                        end_time_j = one_day_later
                    else:
                        end_time_j = datetime.strptime(end_time_j, "%H:%M:%S")

                    # If both time ranges overlap, merge them together. Otherwise,
                    # sort the time ranges while keeping them separate.
                    if start_time_i <= end_time_j and start_time_j <= end_time_i:
                        start_time = min(start_time_i, start_time_j)
                        end_time = max(end_time_i, end_time_j)

                        if end_time == one_day_later:
                            values[i] = f"{start_time.strftime('%H:%M:%S')}-{INVALID_END_TIME}"
                        else:
                            values[i] = f"{start_time.strftime('%H:%M:%S')}-{end_time.strftime('%H:%M:%S')}"

                        values[j] = INVALID_TIME_RANGE
                    elif start_time_i > start_time_j:
                        values[i], values[j] = values[j], values[i]
                elif valid_j:
                    values[i], values[j] = values[j], values[i]

        # Append comment to invalid strings in values
        values = [INVALID_TIME_RANGE_STRING if value == INVALID_TIME_RANGE else value for value in values]

        # Update enable_var and comboboxes based on values after sorting and merging
        for time_range_index, (enable_var, comboboxes) in enumerate(field_time_range):
            value = values[time_range_index]

            if value == INVALID_TIME_RANGE_STRING:
                # Uncheck checkbox and disable comboboxes
                enable_var.set(0)

                for index, combobox in enumerate(comboboxes):
                    if index % 3 == 0:
                        combobox.set("HH")
                    elif index % 3 == 1:
                        combobox.set("MM")
                    else:
                        combobox.set("SS")

                    combobox.config(state="disabled")
            else:
                # Check checkbox and enable and populate comboboxes
                enable_var.set(1)

                time_range = value.split("-")
                for i in range(2):
                    time_values = time_range[i].split(":")

                    if i == 0:
                        is_valid_time = time_values != [INVALID_START_HOUR, INVALID_START_MINUTE, INVALID_START_SECOND]
                    else:
                        is_valid_time = time_values != [INVALID_END_HOUR, INVALID_END_MINUTE, INVALID_END_SECOND]

                    if is_valid_time:
                        for j in range(3):
                            comboboxes[i*3 + j].set(time_values[j])
                    else:
                        comboboxes[i*3].set("HH")
                        comboboxes[i*3 + 1].set("MM")
                        comboboxes[i*3 + 2].set("SS")

                for combobox in comboboxes:
                    combobox.config(state="readonly")

        return values

    def save_config(self):
        """
        Saves the current configuration parameters to an INI file.
        This method iterates through the configuration parameters, processes their values based on their types,
        and saves them to a dictionary. It performs validation and adjustments on specific parameters such as
        frequency cutoffs and date ranges. The final configuration is written to an INI file selected by the user.
        Parameters:
        None
        Returns:
        None
        Raises:
        None
        Notes:
        - The method handles different types of configuration parameters including date ranges, multi-time ranges,
          day-of-week checkboxes, and numeric values.
        - It ensures that the lower and upper cutoff frequencies are valid and within the acceptable range.
        - The system time is adjusted based on the system time offset and saved in the configuration.
        - The configuration is saved to a file named "config.ini" in a directory selected by the user.
        """
        config_values = {}

        system_time_offset = 0

        for entry in self.config_params:

            tab_index = entry["tab_index"]
            param_name = entry["display_name"]
            hover_text = entry["hover_text"]
            ini_field = entry["ini_field"]
            menu_type = entry["menu_type"]
            is_number = entry["is_number"]
            is_integer = entry["is_integer"]
            values = entry["values"]
            default = entry["default"]

            skip_save_ini_field = False

            if menu_type == "date-range-text-box":
                value = self.date_range_text_box_to_ini_value(getattr(self, ini_field))

            elif menu_type == "multi-time-range-text-box":

                values = self.multi_time_range_text_box_to_ini_value(getattr(self, ini_field))

                for time_range_index, value in enumerate(values):
                    config_values[ini_field + "_" + str(time_range_index + 1)] = values[time_range_index]

                # Skip saving the main ini field
                skip_save_ini_field = True

            elif menu_type == "day-of-week-checkboxes":
                value = [var.get() for var in getattr(self, ini_field)]
                value = "".join([day for i, day in enumerate(["M", "T", "W", "R", "F", "S", "U"]) if value[i] == 1])
                if value == "":
                    value = "X # X value indicates no day selected"
            elif menu_type == "checkboxes":
                value = [var.get() for var in getattr(self, ini_field)]
            else:

                value = getattr(self, ini_field).get()

                if is_number:
                    try:
                        value = float(value)

                        if is_integer:
                            value = int(value)

                        # values is a range
                        if values is not None and isinstance(values, tuple):
                            # Clip value to range
                            min_value, max_value = values

                            if min_value is not None:
                                value = max(value, min_value)

                            if max_value is not None:
                                value = min(value, max_value)

                    except ValueError:
                        value = default

                if ini_field == "system_time_offset":
                    if value is not None:
                        system_time_offset = value

                    # Don't save system time offset to config file
                    continue

            if not skip_save_ini_field:
                config_values[ini_field] = value

        # Check if lower frequency is valid and is a number
        try:
            lower_cutoff_frequency = int(config_values[LOWER_CUTOFF_FREQ_INI_FIELD])
            if lower_cutoff_frequency is None or \
                lower_cutoff_frequency == "" or \
                    lower_cutoff_frequency < 0 or \
                    (lower_cutoff_frequency > config_values["sampling_frequency"] // 2):
                config_values[LOWER_CUTOFF_FREQ_INI_FIELD] = 0
            else:
                config_values[LOWER_CUTOFF_FREQ_INI_FIELD] = lower_cutoff_frequency
        except ValueError:
            config_values[LOWER_CUTOFF_FREQ_INI_FIELD] = 0

        # Check if upper frequency is valid and is a number
        try:
            upper_cutoff_frequency = config_values[UPPER_CUTOFF_FREQ_INI_FIELD]
            if upper_cutoff_frequency is None or \
                upper_cutoff_frequency == "" or \
                    upper_cutoff_frequency < 0 or \
                    (upper_cutoff_frequency > config_values["sampling_frequency"] // 2):
                config_values[UPPER_CUTOFF_FREQ_INI_FIELD] = config_values["sampling_frequency"] // 2
            else:
                config_values[UPPER_CUTOFF_FREQ_INI_FIELD] = upper_cutoff_frequency
        except ValueError:
            config_values[UPPER_CUTOFF_FREQ_INI_FIELD] = config_values["sampling_frequency"] // 2

        # Check if lower frequency is lower than upper frequency
        if config_values[LOWER_CUTOFF_FREQ_INI_FIELD] >= config_values[UPPER_CUTOFF_FREQ_INI_FIELD]:
            config_values[LOWER_CUTOFF_FREQ_INI_FIELD] = 0
            config_values[UPPER_CUTOFF_FREQ_INI_FIELD] = config_values["sampling_frequency"] // 2

        # Adjust the lower and upper cutoff frequency text box values
        getattr(self, LOWER_CUTOFF_FREQ_INI_FIELD).delete(0, tk.END)
        getattr(self, LOWER_CUTOFF_FREQ_INI_FIELD).insert(0, str(config_values[LOWER_CUTOFF_FREQ_INI_FIELD]))

        getattr(self, UPPER_CUTOFF_FREQ_INI_FIELD).delete(0, tk.END)
        getattr(self, UPPER_CUTOFF_FREQ_INI_FIELD).insert(0, str(config_values[UPPER_CUTOFF_FREQ_INI_FIELD]))

        if config_values[DATE_RANGE_INI_FIELD].startswith("9999"):
            getattr(self, DATE_RANGE_INI_FIELD)[0].set("YYYY")
            getattr(self, DATE_RANGE_INI_FIELD)[1].set("MM")
            getattr(self, DATE_RANGE_INI_FIELD)[2].set("DD")

        if config_values[DATE_RANGE_INI_FIELD][11:].startswith("9999"):
            getattr(self, DATE_RANGE_INI_FIELD)[3].set("YYYY")
            getattr(self, DATE_RANGE_INI_FIELD)[4].set("MM")
            getattr(self, DATE_RANGE_INI_FIELD)[5].set("DD")

        current_time = datetime.now()
        # Add system time offset to current time
        current_time += timedelta(seconds=system_time_offset)

        time_zone_offset = -time.timezone if (time.localtime().tm_isdst == 0) else -time.altzone
        offset_hours = time_zone_offset // 3600
        offset_minutes = (time_zone_offset % 3600) // 60
        timezone = f"{offset_hours:+03d}:{offset_minutes:02d}"

        system_time_string = current_time.strftime(f"%Y/%m/%d %H:%M:%S {timezone}")
        system_time_string += " # Time must be of the form YYYY/MM/DD HH:MM:SS +/-HH:MM. If it is of the form YYYY/MM/DD HH:MM:SS, then the timezone is assumed to be UTC."
        config_values["system_time"] = system_time_string
        config_values["time_zone"] = timezone + " # Timezone offset in hours and minutes. This can be used to switch to a different timezone than the system timezone."

        # Write the config to an INI file
        config = configparser.ConfigParser()
        config.add_section("Config")
        for key, value in config_values.items():
            config.set("Config", key, str(value))

        # Save as file name "config.ini"
        file_name = "config.ini"

        # Directory must be a volume in the system
        directory = filedialog.askdirectory(
            title="Select SD Card Volume to save configuration file",
            mustexist=True,
            initialdir=".")

        if directory is not None and directory != "":
            file_path = f"{directory}/{file_name}"
            with open(file_path, "w") as config_file:
                config.write(config_file)

            print(f"Config saved successfully to: {file_path}")


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("510x500")
    app = ConfigGeneratorApp(root)
    root.mainloop()
