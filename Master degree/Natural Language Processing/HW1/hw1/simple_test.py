from typing import List, Dict

from stud.implementation import build_model


def main(sentences: List[Dict]):

    model = build_model('cpu')
    predicted_sentences = model.predict(sentences)

    for sentence, predicted_label in zip(sentences, predicted_sentences):
        print(f'# Lemma = {sentence["lemma"]}')

        print(f'# sentence 1 = {sentence["sentence1"]}')
        print(f'# sentence 2 = {sentence["sentence2"]}')

        print(f'# Prediction: {predicted_label}')

        print()


if __name__ == '__main__':
    main([{
            'id': 'test.X',
            'lemma': 'dolor',
            'pos': 'XX',
            'sentence1': 'Lorem ipsum dolor sit amet.',
            'sentence2': 'Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur.',
            'start1': '12',
            'end1': '17',
            'start2': '16',
            'end2': '21'
        }, 
        {
            'id': 'test.X+1',
            'lemma': 'dolor',
            'pos': 'XX',
            'sentence1': 'Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.',
            'sentence2': 'Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur.',
            'start1': '46',
            'end1': '52',
            'start2': '16',
            'end2': '21'} ])
