# Application Global Variables
# This module serves as a way to share variables across different
# modules (global variables).

import os

# Set to True while developing (extra logging in the Text Commands window).
DEBUG = True

# The add-in name is taken from the folder name; used to build unique IDs.
ADDIN_NAME   = os.path.basename(os.path.dirname(__file__))
COMPANY_NAME = 'CycloidRP'  # Used as a prefix for all UI element IDs.