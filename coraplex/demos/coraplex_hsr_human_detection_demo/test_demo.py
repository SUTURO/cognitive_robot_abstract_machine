#!/usr/bin/env python
"""
Run the HSR human detection demo without a viewer or RoboKudo.
"""

from __future__ import annotations

import demo

if __name__ == "__main__":
    demo.main(headless=True, perceives=False)
