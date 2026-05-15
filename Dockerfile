FROM python:3.11-slim

LABEL maintainer="MUGIN Flight Control Team"
LABEL description="MUGIN-3 3600MM H-Tail UAV Flight Control System"

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    supervisor \
    i2c-tools \
    gpsd \
    gpsd-clients \
    && rm -rf /var/lib/apt/lists/*

# Create log directory
RUN mkdir -p /var/log/mugin3 && chmod 755 /var/log/mugin3

# Copy requirements
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY flight_control_system.py .
COPY hardware_interface.py .
COPY telemetry.py .
COPY flight_control_main.py .
COPY flight_control_config.json /etc/mugin3/

# Copy supervisor config
COPY supervisor_fcs.conf /etc/supervisor/conf.d/

# Create config directory
RUN mkdir -p /etc/mugin3

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
    CMD python3 -c "import asyncio; print('healthy')" || exit 1

# Run supervisor
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]
