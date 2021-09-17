from typing import List, Dict

from stud.implementation import build_model


def main(sentences: List[Dict]):

    model = build_model('cpu')
    predicted_sentences_wic, predicted_sentences_wsd = model.predict(sentences)

    for sentence, predicted_label_wic, predicted_label_wsd in zip(sentences, predicted_sentences_wic, predicted_sentences_wsd):
        print(f'# Lemma = {sentence["lemma"]}')

        print(f'# sentence 1 = {sentence["sentence1"]}')
        print(f'# sentence 2 = {sentence["sentence2"]}')

        print(f'# Prediction WiC: {predicted_label_wic}')
        print(f'# Prediction WSD: {predicted_label_wsd}')

        print()


if __name__ == '__main__':
    main([{
            'id': 'test.X',
            'lemma': 'apple',
            'pos': 'NOUN',
            'sentence1': 'Lorem ipsum apple sit amet.',
            'sentence2': 'Duis aute irure apple in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur.',
            'start1': '12',
            'end1': '17',
            'start2': '16',
            'end2': '21'
        }, 
        {
            'id': 'test.X+1',
            'lemma': 'apple',
            'pos': 'NOUN',
            'sentence1': 'Sed do eiusmod tempor incididunt ut labore et applee magna aliqua.',
            'sentence2': 'Duis aute irure apple in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur.',
            'start1': '46',
            'end1': '52',
            'start2': '16',
            'end2': '21'} ])
