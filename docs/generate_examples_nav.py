from pathlib import Path

import mkdocs_gen_files

# Start with the main examples entry
nav = mkdocs_gen_files.Nav()

# Path to examples directory
examples_dir = Path('docs/examples')

# First, add individual MD files in the root of examples directory
for md_file in sorted(examples_dir.glob('*.md')):
    # Use the filename only, not prefixed with examples/
    rel_path = md_file.name
    nav[md_file.stem.replace('-', ' ').title()] = rel_path

# Then add README.md files from subdirectories
for subdir in sorted(examples_dir.glob('*/')):
    if subdir.is_dir():
        readme_path = subdir / 'README.md'
        if readme_path.exists():
            # Use subdirectory/README.md without examples/ prefix
            rel_path = f'{subdir.name}/README.md'
            nav[subdir.name.replace('_', ' ').title()] = rel_path

# Write navigation to SUMMARY file
with mkdocs_gen_files.open('examples/SUMMARY.md', 'w') as nav_file:
    nav_file.writelines(nav.build_literate_nav())
