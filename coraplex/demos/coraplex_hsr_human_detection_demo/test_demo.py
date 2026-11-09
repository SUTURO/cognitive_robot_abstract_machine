#!/usr/bin/env python
"""
Run the HSR human detection demo without a viewer and without RoboKudo; a failure exits
non-zero with its traceback.
"""

import demo

if __name__ == "__main__":
    demo.main(headless=True, perceives=False)
