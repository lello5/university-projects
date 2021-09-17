from typing import List, Dict

class Model:

    def predict(self, sentence_pairs: List[Dict]) -> List[Dict]:
        """
        A simple wrapper for your model

        Args:
            sentence_pairs: list of dicts of strings. The outer list represents the samples, the inner one a dictionary with sentence pairs and extra information such as the lemma for WiC and the indexes in context in the two sentences.
            within it. Ex: [{
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
                                'end2': '21'} ]

        Returns:
            list of predictions associated to each sentence pair.
            Ex: Ex: [ "True", "False" ]

        """
        raise NotImplementedError
