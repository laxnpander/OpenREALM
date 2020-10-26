import os
import argparse
import re
import statistics

def process_log(log_file):
	# Create a dictionary with all timed operations
	timed_functions = {}

	# Go through the log file and find all infos that start with "Timing"
	lines = log_file.readlines()
	for line in lines:
		timing_info = re.search('Timing (.*) ms', line)

		# Not all lines have timing info, for those that have
		if timing_info is not None:
			s = timing_info.group(1)
			function_name = s[s.find("[") + 1:s.find("]")]
			time =  int(s[s.find(": ") + 1:])

			if function_name in timed_functions:
				timed_functions[function_name].append(time)
			else:
				timed_functions[function_name] = [time]

	for f in timed_functions:
		data = timed_functions[f]
		print(" ")
		print("### Processing step [" + f + "] ###")
		print("Average: " + str(statistics.mean(data)) + " ms")
		print("StdDev: " + str(statistics.stdev(data)) + " ms")
		print("Min: " + str(min(data)) + "ms")
		print("Max: " + str(max(data)) + "ms")



if __name__ == "__main__":
	parser = argparse.ArgumentParser(
		description="Script that extracts timing informations from a stage log.",
	)

	# Add the arguments:
	#   - log_file: full path to the log file to be processed.

	parser.add_argument(
		'log_file',
		help='The location of the log file '
	)

	args = parser.parse_args()

	log_file = args.log_file

	# Read the log file
	try:
	  f = open(log_file, 'r')
	except FileNotFoundException:
	  print('Error: Could log file does not exist at %s', log_file)
	  raise SystemExit

	process_log(f)

	f.close()