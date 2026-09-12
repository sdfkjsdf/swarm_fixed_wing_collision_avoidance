"""Decode complete shutdown-only CDR dumps. Never imported by flight nodes."""
from pathlib import Path


def parse(text):
    blocks, current, seen = [], None, set()
    for line in text.splitlines():
        if not line.startswith('[stop-observation'):
            continue
        fields = line.split(',')
        if fields[0] == '[stop-observation-begin]':
            if current is not None or len(fields) != 8 or fields[1] != '1':
                raise ValueError('Invalid/nested stopped observation header')
            _, _, vehicle, owner, topic, message_type, count, dropped = fields
            key = (int(vehicle), owner)
            if key in seen or min(int(count), int(dropped)) < 0:
                raise ValueError('Duplicate stream or negative count')
            seen.add(key)
            current = dict(vehicle=int(vehicle), owner=owner, topic=topic,
                           message_type=message_type, count=int(count),
                           dropped=int(dropped), records=[])
        elif fields[0] == '[stop-observation]':
            if current is None or len(fields) != 3 or int(fields[1]) <= 0:
                raise ValueError('Record outside stream or invalid time')
            payload = bytes.fromhex(fields[2])
            if not payload:
                raise ValueError('Empty CDR')
            current['records'].append((int(fields[1]), payload))
        elif fields[0] == '[stop-observation-end]':
            if (current is None or len(fields) != 4
                    or (int(fields[1]), fields[2], int(fields[3])) !=
                    (current['vehicle'], current['owner'], current['count'])
                    or len(current['records']) != current['count']):
                raise ValueError('Incomplete or mismatched stopped observation stream')
            blocks.append(current)
            current = None
        else:
            raise ValueError('Unknown stopped observation record')
    if current is not None:
        raise ValueError('Missing stopped observation footer')
    return blocks


def merge_into(messages, log_dir):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    metadata, additions = [], {}
    if log_dir is None:
        return metadata
    for path in sorted(Path(log_dir).glob('guidance_*.log')):
        for block in parse(path.read_text()):
            topic = block['topic']
            if messages.get(topic):
                raise ValueError(f'Live and stopped diagnostics overlap: {topic}')
            cls = get_message(block['message_type'])
            additions.setdefault(topic, []).extend(
                (stamp, deserialize_message(payload, cls))
                for stamp, payload in block['records'])
            metadata.append({k: v for k, v in block.items() if k != 'records'})
    for topic, records in additions.items():
        messages[topic] = sorted(records, key=lambda record: record[0])
    return metadata
