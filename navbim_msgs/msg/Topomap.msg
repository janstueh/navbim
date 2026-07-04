# Complete topological map definition

# Metadata
string source_file           # Original JSON file path
string building_name         # Building identifier
uint32 total_floors         # Number of floors in building
string[] floor_names        # List of floor names

# Graph structure
TopomapNodes nodes
TopomapEdges edges