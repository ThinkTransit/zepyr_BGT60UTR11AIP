#!/bin/bash
# BGT60 Radar UI Setup Script

echo "Setting up BGT60 Radar UI..."

# Check if we're in a pipenv environment
if [[ -n "$PIPENV_ACTIVE" ]]; then
    echo "Detected pipenv environment, installing directly..."
    pipenv install PyQt6 pyserial numpy matplotlib
else
    # Try to create virtual environment
    if python3 -m venv radar_ui_env 2>/dev/null; then
        echo "Created virtual environment"
        source radar_ui_env/bin/activate
        pip install -r requirements.txt
    else
        echo "Virtual environment creation failed, installing with pip..."
        pip install --user -r requirements.txt
    fi
fi

echo "Setup complete!"
echo ""
echo "To run the UI:"
if [[ -n "$PIPENV_ACTIVE" ]]; then
    echo "Run: python radar_ui.py"
elif [[ -f "radar_ui_env/bin/activate" ]]; then
    echo "1. Activate environment: source radar_ui_env/bin/activate"
    echo "2. Run UI: python radar_ui.py"
else
    echo "Run: python radar_ui.py"
fi
echo ""
echo "Make sure your radar board is connected to USB and appears as /dev/ttyUSB0 or similar"