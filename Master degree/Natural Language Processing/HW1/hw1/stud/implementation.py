import collections
from collections import Counter, defaultdict
import numpy as np
import jsonlines
import os
import re
from typing import *
import torch
from torch import nn
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
torch.manual_seed(1792507)
from model import Model
    
# PRE-TRAINED EMBEDDING (Word2Vec Continuous Skipgram - English CoNLL17 corpus)
word_vectors = dict()
words_limit = 100_000
with open('model/word2vec.txt', 'r', encoding="ISO-8859-1") as f:
    next(f)  # skip header
    for i, line in enumerate(f):
        if i == words_limit:
            break
        word, *vector = line.strip().split(' ')
        vector = torch.tensor([float(c) for c in vector])
        word_vectors[word] = vector

# VECTORS_STORE AND WORD_INDEX TO DEAL WITH UNKNOWN AND PADDING
word_index = dict()
vectors_store = []

vectors_store.append(torch.rand(100))           # index = 0 -> pad token
vectors_store.append(torch.rand(100))           # index = 1 -> unk token
vectors_store.append(torch.rand(100))           # index = 2 -> separator token (between two sentences in second apporach)

for word, vector in word_vectors.items():
    word_index[word] = len(vectors_store)       # index = 3, 4 .....
    vectors_store.append(vector)                # third, forth... words

word_index = defaultdict(lambda: 1, word_index) # unknown word -> default index = 1
vectors_store = torch.stack(vectors_store)

# line -> extract features and append to data_store
def vectorize_line(line):
    vector = []

    vectorize_sentence('sentence1', line, vector)    # vectorize and append first sentence
    vector.append(vectors_store[0])                         # sentences separator; in this case I used the pad token because I used the token separator only in second approach
    vectorize_sentence('sentence2', line, vector)    # vectorize and append second sentence
    
    vector = torch.mean(torch.stack(vector), dim=0)    # data as mean of the vector components
    # In my notebook I use the real supervised label, in this case it is not passed
    # I am using this "fake" label to keep the same code structure
    label = 0                       
    return (vector, label)

# sentence -> extract feature of single sentence
def vectorize_sentence (sentence: str, line, vector):
    for word in line[sentence].strip().split(' '):
        word = re.sub('[\.,:;!@#$\(\)\-&\\<>/0-9"”“]', '', word).lower()
        if len(word) > 3:
          vector.append(vectors_store[word_index[word]])

def build_model(device: str) -> Model:
    # STUDENT: return StudentModel()
    # STUDENT: your model MUST be loaded on the device "device" indicates
    # PRE-TRAINED EMBEDDING (Word2Vec Continuous Skipgram - English CoNLL17 corpus)
    
    model = StudentModel(100, 1024)
    model.eval()
    model.load_state_dict(torch.load('model/saved_model_0.667.pt', map_location=torch.device(device)))

    return model.to(device)


class RandomBaseline(Model):

    options = [
        ('True', 40000),
        ('False', 40000),
    ]

    def __init__(self):

        self._options = [option[0] for option in self.options]
        self._weights = np.array([option[1] for option in self.options])
        self._weights = self._weights / self._weights.sum()

    def predict(self, sentence_pairs: List[Dict]) -> List[str]:
        return [str(np.random.choice(self._options, 1, p=self._weights)[0]) for x in sentence_pairs]

class Model(torch.nn.Module):
    pass

class StudentModel(Model):
    # STUDENT: construct here your model
    # this class should be loading your weights and vocabulary
    def __init__(self, n_features: int, n_hidden: int):
        super().__init__()
        # Linear layers
        self.lin1 = torch.nn.Linear(n_features, n_hidden)
        self.lin2 = torch.nn.Linear(n_hidden, n_hidden//2)
        self.lin3 = torch.nn.Linear(n_hidden//2, n_hidden//4)
        self.lin4 = torch.nn.Linear(n_hidden//4, n_hidden//8)
        self.lin5 = torch.nn.Linear(n_hidden//8, 1)
        # Dropout layer
        self.drop = torch.nn.Dropout(0.4)
        # Binary classification task -> Binary cross-entropy loss function
        self.loss_fn = torch.nn.BCELoss()

    def forward(self, x: torch.Tensor, y: Optional[torch.Tensor] = None) -> Dict[str, torch.Tensor]:
        # Network structure
        out = self.lin1(x)
        out = torch.relu(out)
        out = self.drop(out)
        out = self.lin2(out)
        out = torch.relu(out)
        out = self.drop(out)
        out = self.lin3(out)
        out = torch.relu(out)
        out = self.drop(out)
        out = self.lin4(out)
        out = torch.relu(out)
        out = self.drop(out)
        out = self.lin5(out).squeeze(1)
        # Binary classification task -> Sigmoid activation function
        out = torch.sigmoid(out)

        result = {'pred': out}

        # If we have labels, loss computation
        if y is not None:
            loss = self.loss(out, y)
            result['loss'] = loss
        return result

    def loss(self, pred, y):
        return self.loss_fn(pred, y)

    def predict(self, sentence_pairs: List[Dict]) -> List[str]:
        # STUDENT: implement here your predict function
        # remember to respect the same order of sentences!
        data_store = []
        for line in sentence_pairs:
            data_store.append(vectorize_line(line))

        validation_dataloader = DataLoader(data_store, batch_size=32)
        res = []
        for x, _ in validation_dataloader:
            batch_out = self(x)
            pred = torch.round(batch_out['pred'])
            for elem in pred:
                if elem>0.5:
                  res.append('True')
                else:
                  res.append('False')
        return res
