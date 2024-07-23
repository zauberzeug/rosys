#!/usr/bin/env python3
import argparse
import re
import sys

import requests

BASE_URL = 'https://api.github.com/repos/zauberzeug/rosys'

parser = argparse.ArgumentParser(description='Fetch the content of a milestone from a GitHub repo.')
parser.add_argument('milestone_title', help='Title of the milestone to fetch.')
args = parser.parse_args()
milestone_title: str = args.milestone_title

milestones = requests.get(f'{BASE_URL}/milestones?state=all', timeout=5).json()
matching_milestones = [milestone for milestone in milestones if milestone['title'] == milestone_title]
if not matching_milestones:
    print(f'Milestone "{milestone_title}" not found!')
    sys.exit(1)
milestone_number = matching_milestones[0]['number']

issues = requests.get(f'{BASE_URL}/issues?milestone={milestone_number}&state=all', timeout=5).json()
sections: dict[str, list[str]] = {
    'New features and enhancements': [],
    'Bugfixes': [],
    'Documentation': [],
    'Others': [],
}
for issue in issues:
    title: str = issue['title']
    user: str = issue['user']['login']
    body: str = issue['body'] or ''
    labels: list[str] = [label['name'] for label in issue['labels']]
    number_patterns = [r'#(\d+)', r'https://github.com/zauberzeug/rosys/(?:issues|discussions|pulls)/(\d+)']
    numbers = [issue['number']] + [int(match) for pattern in number_patterns for match in re.findall(pattern, body)]
    numbers_str = ', '.join(f'#{number}' for number in sorted(numbers))
    note = f'{title.strip()} ({numbers_str} by @{user})'
    if 'bug' in labels:
        sections['Bugfixes'].append(note)
    elif 'enhancement' in labels:
        sections['New features and enhancements'].append(note)
    elif 'documentation' in labels:
        sections['Documentation'].append(note)
    else:
        sections['Others'].append(note)

for title, notes in sections.items():
    if not notes:
        continue
    print(f'### {title}')
    print()
    for note in notes:
        print(f'- {note}')
    print()
