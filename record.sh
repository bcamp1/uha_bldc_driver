#!/bin/bash
set -v
date_str=$(date +"%-m-%-d-%y_h%Hm%Ms%S")
recent_str="recent"
UHA=$(pwd)

# Data files
bin_file="$UHA/logs/bin/$date_str.bin"
csv_file="$UHA/logs/csv/$date_str.csv"
plot_file="$UHA/logs/plots/$date_str.png"

recent_bin_file="$UHA/logs/bin/$recent_str.bin"
recent_csv_file="$UHA/logs/csv/$recent_str.csv"
recent_plot_file="$UHA/logs/plots/$recent_str.png"

# Python scripts
bin2csv_file="$UHA/logs/bin2csv.py"
csv2plot_file="$UHA/logs/csv2plot.py"

# Record UART data
echo "Recording to $csv_file... (Ctrl-C to stop)"
stty -F /dev/ttyUSB0 -echo
timeout 4s socat -d -d -u FILE:/dev/ttyUSB0,b115200,raw STDOUT > "$csv_file"

# Clean up CSV file
echo "Cleaning up CSV file..."
sed -i 's/\r//g' "$csv_file"
sed -i '1d;$d' "$csv_file"
cp "$csv_file" "$recent_csv_file"

# Create CSV
# echo "Parsing binary to CSV..."
# python "$bin2csv_file" -i "$bin_file"
# cp "$csv_file" "$recent_csv_file"
# 
# echo "Bin count: $(wc -c < $bin_file) bytes"
echo "CSV count: $(wc -l < $csv_file) entries"

# Create plot
echo "Generating plot from CSV..."
python "$csv2plot_file" -i "$csv_file"
cp "$plot_file" "$recent_plot_file"

# Optionally show plot
if [ "$1" = "-f" ]; then
    echo "Showing plot..."
    pinta "$recent_plot_file" > /dev/null 2>&1
fi

