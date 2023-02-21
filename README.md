# Low Cost Thermodenuder

Working with the LAQR at Colorado State University this is the code for the central processor for a low cost thermodenuder. Uses a simple PID loop to control the PWM of an AC relay connecting to some heat tape based on the temperature reading of two embedded thermocouples (MCP9600). It then uploads this info to a custom UBIDOTS page for control/monitoring
