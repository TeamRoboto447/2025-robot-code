import os
import threading
from networktables import NetworkTables, NetworkTablesInstance
from ntcore import *
from networktables.util import ntproperty
from PIL import Image, ImageDraw, ImageFont
from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.ImageHelpers import PILHelper
from time import sleep

# import logging
# logging.basicConfig(level=logging.DEBUG)
LIVE = True

# Folder location of image assets used by this example.
ASSETS_PATH = os.path.join(os.path.dirname(__file__), "Assets")

nt = NetworkTablesInstance.getDefault()
# Set up NetworkTables
if LIVE:
    nt.initialize(server=("10.4.47.2", 1735))
else:
    # If not connected in the live environment, we need to be in host mode
    # nt = NetworkTables.initialize()
    nt.initialize(server="127.0.0.1")

print("Waiting For Robot Connection")
while not NetworkTables.isConnected():
    sleep(0.25)

smartdashboard_table = nt.getTable("SmartDashboard")  # Interact w/ smartdashboard
steamdeck_root = nt.getTable(
    "Streamdeck-Control"
)  # Make table for values to be changed from this script
panel_network_table = steamdeck_root.getSubTable("panel")
robot_requesting_update_entry = panel_network_table.getEntry("robotRequestingUpdate")
button_table = panel_network_table.getSubTable("states")


class Button:
    def __init__(
        self,
        functionality="HOLD",
        label="",
        enabled_icon="green.png",
        disabled_icon="empty.png",
    ):
        self.label = label
        self.functionality = functionality
        self.__state = False
        self.__enabled_icon = enabled_icon
        self.__disabled_icon = disabled_icon

    @property
    def icon(self):
        if self.__state:
            return self.__enabled_icon
        return self.__disabled_icon

    def set_enabled_icon(self, icon):
        self.__enabled_icon = icon

    def set_disabled_icon(self, icon):
        self.__disabled_icon = icon

    def set_mirrored_icons(self, icon):
        self.__enabled_icon = icon
        self.__disabled_icon = icon

    @property
    def state(self):
        return self.__state

    @property
    def label(self):
        return self.__label

    @label.setter
    def label(self, label):
        if len(label) > 15:
            raise ValueError("Label can't be longer than 15 characters")
        self.__label = label

    @property
    def functionality(self):
        return self.__functionality

    @functionality.setter
    def functionality(self, new_value):
        new_value = new_value.upper()
        if not (new_value == "TOGGLE") and not (new_value == "HOLD"):
            raise ValueError('Button functionality must be either "TOGGLE" or "HOLD"')
        self.__functionality = new_value

    def trigger(self, state):
        if self.__functionality == "TOGGLE" and state:
            self.__state = not self.__state
        elif self.__functionality == "HOLD":
            self.__state = state
        button_table.putBoolean(self.__label, self.__state)

    def reset(self):
        self.__state = False
        button_table.putBoolean(self.__label, self.__state)

    def reset_with_new_args(
        self, functionality="", label="", enabled_icon="", disabled_icon=""
    ):
        self.reset()
        if not (functionality == ""):
            self.functionality = functionality

        if not (label == ""):
            self.label = label

        if not (enabled_icon == ""):
            self.__enabled_icon = enabled_icon

        if not (disabled_icon == ""):
            self.__disabled_icon = disabled_icon


class ControlPanel:
    def __init__(self):
        self.__buttons = {}
        self.deck_reference = None
        for x in range(5):
            for y in range(3):
                self.init_button_by_pos(x, y)
        self.update_network_tables()

    def update_network_tables(self):
        panel_network_table.putStringArray("labels", self.get_labels_as_list())
        for button in self.__buttons.values():
            button_table.putBoolean(button.label, button.state)

    def get_labels(self):
        labels = {}
        for x in range(5):
            for y in range(3):
                labels[(x, y)] = self.get_label_by_pos(x, y)
        return labels

    def get_labels_as_list(self):
        label_dict = self.get_labels()
        return [
            label_dict[key]
            for key in sorted(label_dict.keys(), key=lambda t: (t[0] + (t[1] * 5)))
        ]

    def get_label_by_pos(self, x, y):
        return self.get_button(x, y).label

    def reset_button_with_new_args(
        self, x, y, functionality="", label="", enabled_icon="", disabled_icon=""
    ):
        self.get_button(x, y).reset_with_new_args(
            functionality, label, enabled_icon, disabled_icon
        )
        update_key_image(self.deck_reference, (x + (y * 5)), False)

    def init_button_by_pos(self, x, y):
        button = Button("HOLD", f"Button {(x + (y * 5))}")
        self.__buttons[(x, y)] = button

    def enable_semi_auto_layout(self):
        self.reset_button_with_new_args(0, 0, "HOLD", "Floor Collect", "processor.jpeg", "processor.jpeg")
        self.reset_button_with_new_args(1, 0, "HOLD", "Algae Processor", "processor.jpeg", "processor.jpeg")
        self.reset_button_with_new_args(2, 0, "HOLD", "Algae Net", "net.png", "net.png")
        self.reset_button_with_new_args(3, 0, "HOLD", "Algae L1", "processor.jpeg", "processor.jpeg")
        self.reset_button_with_new_args(4, 0, "HOLD", "Algae L2", "processor.jpeg", "processor.jpeg")
        self.reset_button_with_new_args(0, 1, "HOLD", "Coral Trough", "L1.jpeg", "L1.jpeg")
        self.reset_button_with_new_args(1, 1, "HOLD", "Coral L2", "L2.jpeg", "L2.jpeg")
        self.reset_button_with_new_args(2, 1, "HOLD", "Coral Loading", "coral_intake.png", "coral_intake.png")
        self.reset_button_with_new_args(3, 1, "HOLD", "Shift Forward", "front.jpeg", "front.jpeg")
        self.reset_button_with_new_args(4, 1, "HOLD", "L3 w/ Algae", "processor.jpeg", "processor.jpeg")
        self.reset_button_with_new_args(0, 2, "HOLD", "Coral L3", "L3.jpeg", "L3.jpeg")
        self.reset_button_with_new_args(1, 2, "HOLD", "Coral L4", "L4.jpeg", "L4.jpeg")
        self.reset_button_with_new_args(2, 2, "HOLD", "Shift Left", "left.jpeg", "left.jpeg")
        self.reset_button_with_new_args(3, 2, "HOLD", "Shift Back", "back.jpeg", "back.jpeg")
        self.reset_button_with_new_args(4, 2, "HOLD", "Shift Right", "right.jpeg", "right.jpeg")

    @property
    def buttons(self):
        # Returns a copy of the list, the list itself is readonly, but the button references themselves are still able to be modified
        return self.__buttons.copy()

    def get_button(self, x, y):
        return self.__buttons[(x, y)]

    def get_button_value(self, x, y):
        return self.__buttons[(x, y)].state

    def get_button_by_index(self, index):
        return self.get_button(index % 5, index // 5)

    def get_button_value_by_index(self, index):
        return self.get_button_value(index % 5, index // 5)


panel = ControlPanel()


# Generates a custom tile with run-time generated text and custom image via the
# PIL module.
def render_key_image(deck, icon_filename, font_filename, label_text):
    # Resize the source image asset to best-fit the dimensions of a single key,
    # leaving a margin at the bottom so that we can draw the key title
    # afterwards.
    icon = Image.open(icon_filename)
    image = PILHelper.create_scaled_key_image(deck, icon, margins=[0, 0, 20, 0])

    # Load a custom TrueType font and use it to overlay the key index, draw key
    # label onto the image a few pixels from the bottom of the key.
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype(font_filename, 14)
    draw.text(
        (image.width / 2, image.height - 5),
        text=label_text,
        font=font,
        anchor="ms",
        fill="white",
    )

    return PILHelper.to_native_key_format(deck, image)


# Returns styling information for a key based on its position and state.
def get_key_style(deck, key, state):
    name = "boolean"
    icon = panel.get_button_by_index(key).icon
    font = "Roboto-Regular.ttf"
    # label = "Pressed!" if state else panel.get_button_by_index(key).label
    label = panel.get_button_by_index(key).label

    return {
        "name": name,
        "icon": os.path.join(ASSETS_PATH, icon),
        "font": os.path.join(ASSETS_PATH, font),
        "label": label,
    }


# Creates a new key image based on the key index, style and current key state
# and updates the image on the StreamDeck.
def update_key_image(deck, key, state):
    # Determine what icon and label to use on the generated key.
    key_style = get_key_style(deck, key, state)

    # Generate the custom key with the requested image and label.
    image = render_key_image(
        deck, key_style["icon"], key_style["font"], key_style["label"]
    )

    # Use a scoped-with on the deck to ensure we're the only thread using it
    # right now.
    with deck:
        # Update requested key with the generated image.
        deck.set_key_image(key, image)


# Prints key state change information, updates rhe key image and performs any
# associated actions when a key is pressed.
def key_change_callback(deck, key, state):
    # Print new key state

    button = panel.get_button_by_index(key)
    button.trigger(state)

    # Don't try to draw an image on a touch button
    if key >= deck.key_count():
        return

    # Update the key image based on the new key state.
    update_key_image(deck, key, state)

    # Check if the key is changing to the pressed state.
    if state:
        key_style = get_key_style(deck, key, state)

        # When an exit button is pressed, close the application.
        if key_style["name"] == "exit":
            # Use a scoped-with on the deck to ensure we're the only thread
            # using it right now.
            with deck:
                # Reset deck, clearing all button images.
                deck.reset()

                # Close deck handle, terminating internal worker threads.
                deck.close()


def valueChanged(table, key, value, isNew):
    print("valueChanged: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
    if key == "robotRequestingUpdate" and value:
        panel.update_network_tables()
        robot_requesting_update_entry.setBoolean(False)


def main_func():
    global panel

    streamdecks = DeviceManager().enumerate()

    print("Found {} Stream Deck(s).\n".format(len(streamdecks)))

    panel_network_table.addEntryListener(valueChanged)
    robot_requesting_update_entry.setBoolean(True)

    for index, deck in enumerate(streamdecks):
        # This example only works with devices that have screens.
        if not deck.is_visual():
            continue

        deck.open()
        deck.reset()
        panel.deck_reference = deck
        panel.enable_semi_auto_layout()

        print(
            "Opened '{}' device (serial number: '{}', fw: '{}')".format(
                deck.deck_type(), deck.get_serial_number(), deck.get_firmware_version()
            )
        )

        # Set initial screen brightness to 30%.
        deck.set_brightness(30)

        # Set initial key images.
        for key in range(deck.key_count()):
            update_key_image(deck, key, False)

        # Register callback function for when a key state changes.
        deck.set_key_callback(key_change_callback)

        # Wait until all application threads have terminated (for this example,
        # this is when all deck handles are closed).
        for t in threading.enumerate():
            try:
                t.join()
            except RuntimeError:
                pass


if __name__ == "__main__":
    try:
        main_func()
    except KeyboardInterrupt:
        if panel.deck_reference:
            panel.deck_reference.reset()
            panel.deck_reference.close()
