import pstats

# Load the profile data
p = pstats.Stats('profile_output.prof')
p.sort_stats('cumulative').print_stats(15)