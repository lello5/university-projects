import logging

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

import argparse
import requests
import time
import jsonlines

from requests.exceptions import ConnectionError
from sklearn.metrics import precision_score, recall_score, f1_score, accuracy_score
from tqdm import tqdm
from typing import Tuple, List, Any, Dict

def count(l: List[Any]) -> Dict[Any, int]:
    d = {}
    for e in l:
        d[e] = 1 + d.get(e, 0)
    return d

def read_dataset(path: str) -> Tuple[List[Dict], List[str]]:

    sentence_pairs = []
    labels = []

    with jsonlines.open(path) as f:
        for obj in f:
            labels.append(obj.pop('label'))
            sentence_pairs.append(obj)

    assert len(sentence_pairs) == len(labels)

    return sentence_pairs, labels

def main(test_path: str, endpoint: str, batch_size=32):

    try:
        sentence_pairs, labels = read_dataset(test_path)
    except FileNotFoundError as e:
        logging.error(f'Evaluation crashed because {test_path} does not exist')
        exit(1)
    except Exception as e:
        logging.error(f'Evaluation crashed. Most likely, the file you gave is not in the correct format')
        logging.error(f'Printing error found')
        logging.error(e, exc_info=True)
        exit(1)

    max_try = 10
    iterator = iter(range(max_try))

    while True:

        try:
            i = next(iterator)
        except StopIteration:
            logging.error(f'Impossible to establish a connection to the server even after 10 tries')
            logging.error('The server is not booting and, most likely, you have some error in build_model or StudentClass')
            logging.error('You can find more information inside logs/. Checkout both server.stdout and, most importantly, server.stderr')
            exit(1)

        logging.info(f'Waiting 10 second for server to go up: trial {i}/{max_try}')
        time.sleep(10)

        try:
            response = requests.post(endpoint, json={'sentence_pairs': [{'id': 'test.X',
                                                                        'lemma': 'dolor',
                                                                        'pos': 'XX',
                                                                        'sentence1': 'Lorem ipsum dolor sit amet.',
                                                                        'sentence2': 'Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur.',
                                                                        'start1': '12',
                                                                        'end1': '17',
                                                                        'start2': '16',
                                                                        'end2': '21'}]}).json()
            response['predictions_s']
            logging.info('Connection succeded')
            break
        except ConnectionError as e:
            continue
        except KeyError as e:
            logging.error(f'Server response in wrong format')
            logging.error(f'Response was: {response}')
            logging.error(e, exc_info=True)
            exit(1)

    predictions_s = []

    progress_bar = tqdm(total=len(sentence_pairs), desc='Evaluating')

    for i in range(0, len(sentence_pairs), batch_size):
        batch = sentence_pairs[i: i + batch_size]
        try:
            response = requests.post(endpoint, json={'sentence_pairs': batch}).json()
            predictions_s += response['predictions_s']
        except KeyError as e:
            logging.error(f'Server response in wrong format')
            logging.error(f'Response was: {response}')
            logging.error(e, exc_info=True)
            exit(1)
        progress_bar.update(len(batch))

    progress_bar.close()

    label_distribution = count(labels)
    pred_distribution = count(predictions_s)

    print(f'# instances: {len(labels)}')

    keys = set(label_distribution.keys()) | set(pred_distribution.keys())
    for k in keys:
        print(f'\t# {k}: ({label_distribution.get(k, 0)}, {pred_distribution.get(k, 0)})')

    p = precision_score(labels, predictions_s, average='macro')
    r = recall_score(labels, predictions_s, average='macro')
    f = f1_score(labels, predictions_s, average='macro')
    a = accuracy_score(labels, predictions_s)

    print(f'# precision: {p:.4f}')
    print(f'# recall: {r:.4f}')
    print(f'# f1: {f:.4f}')
    print(f'# acc: {a:.4f}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=str, help='File containing data you want to evaluate upon')
    args = parser.parse_args()

    main(
        test_path=args.file,
        endpoint='http://127.0.0.1:12345'
    )
