from model import Model

import os
import numpy as np
from typing import Optional, List, Dict, Union, Tuple
import jsonlines
import xml.etree.ElementTree as ET
import pickle

import torch
from torch.utils.data import Dataset, DataLoader
import torchmetrics
import pytorch_lightning as pl
from transformers import BertTokenizer, BertModel, BertPreTrainedModel, BertConfig, AdamW
import nltk
nltk.download('wordnet')
from nltk.corpus import wordnet as wn

pos_map = {"NOUN": wn.NOUN, "VERB": wn.VERB, "ADJ": wn.ADJ, "ADV": wn.ADV}

# Parameters
pretrained_bert =  'bert-base-uncased'
learning_rate = 2e-5
epsilon = 1e-8
hidden_size = 768 if pretrained_bert == 'bert-base-uncased' else 1024
dropout = 0.1

# Initializing a BertTokenizer, used both for preprocessing and for the model
tokenizer = BertTokenizer.from_pretrained(pretrained_bert, do_lower_case=True)
# Using second pre-processing approach
tokenizer.add_special_tokens({'additional_special_tokens': ['[TRG]']})


def build_model(device: str) -> Model:
    # STUDENT: return StudentModel()
    # STUDENT: your model MUST be loaded on the device "device" indicates
    #return RandomBaseline()
    
    global Device # In my notebook the device is a global variable
    Device = torch.device(device)
    
    config = BertConfig.from_pretrained(pretrained_bert, hidden_dropout_prob=dropout, hidden_size=hidden_size)
    model = StudentModel(config)
    model.load_state_dict(torch.load('model/66.9%_bert-base-uncased_error.pth', map_location=Device))
    model.eval().to(Device)

    return model


class RandomBaseline(Model):

    def __init__(self):
        # Load your models/tokenizer/etc that only needs to be loaded once when doing inference
        pass

    def predict(self, sentence_pairs: List[Dict]) -> Tuple[List[str], List[str]]:
        preds_wsd = [(np.random.choice(wn.synsets(pair["lemma"], pos_map[pair["pos"]])).lemmas()[0].key(), np.random.choice(wn.synsets(pair["lemma"], pos_map[pair["pos"]])).lemmas()[0].key()) for pair in sentence_pairs]
        preds_wic = []
        for pred in preds_wsd:
            if pred[0] == pred[1]:
                preds_wic.append('True')
            else:
                preds_wic.append('False')
        return preds_wic, preds_wsd

# -------------------------------------------------- My Model ----------------------------------------------------
class My_BERT(BertPreTrainedModel):
  def __init__(self, config):
    super().__init__(config)
    
    self.bert = BertModel(config)
    self.dropout = torch.nn.Dropout(dropout)
    self.output1 = torch.nn.Linear(hidden_size, hidden_size//2)
    self.output2 = torch.nn.Linear(hidden_size//2, 1)
    self.init_weights()
    
  def forward(self, ids, attention_masks, ids_categories):
    bert_out = self.bert(ids, attention_masks, ids_categories)[1]
    dropout_out = self.dropout(bert_out)
    out1 = self.output1(dropout_out)
    dropout_out = self.dropout(out1)
    return self.output2(dropout_out).squeeze(-1)

class StudentModel(Model, pl.LightningModule):
  def __init__(self, config):
    super().__init__()

    self.bert = My_BERT.from_pretrained(pretrained_bert, config=config)
    self.bert.resize_token_embeddings(len(tokenizer))
    self.save_hyperparameters()
  
  def configure_optimizers(self):
    return AdamW(self.parameters(), lr=learning_rate, eps=epsilon)
  
  # ----------------------------------------------- FORWARD AND LEARNING PROCESS -----------------------------------------------
  def forward(self, ids, attention_masks, ids_categories, labels, is_training: Optional[bool] = False):
    forward_out = self.bert(ids, attention_masks, ids_categories)
    # If I do not know the labels for this batch I only output the raw predictions, this is triggered only in predict
    if labels is None:
      return forward_out
    label = torch.max(labels, -1).indices # gives the index of the right label
    forward_loss = torch.nn.CrossEntropyLoss()(forward_out.unsqueeze(0), label.unsqueeze(0))
    # If I am training I do not need information other than the loss
    if is_training:
      return forward_loss
    # Return this if validating or testing
    return {'model_logits': forward_out, 'label': label, 'forward_loss': forward_loss}

  # Step and epoch_end functions do the same work in train, validation and test..
  # .. only training functions have been reduced to avoid gpu overload
  def training_step(self, batches, batch_id):
    batches_loss = 0
    for batch in batches:
      batches_loss += self.forward(*batch, is_training = True)
    # Mean loss of batches
    loss = batches_loss / (len(batches))
    if log_stuff:
      self.log('train_loss', loss, prog_bar=True)
    return loss

  # training_epoch_end function removed to lighten the training process (gpu crash)

  def validation_step(self, batches, batch_id):
    batches_loss = 0
    prediction = []
    labels = []
    for batch in batches:
      batch_out = self.forward(*batch)
      batches_loss += batch_out['forward_loss']
      # Validating and testing I also extract the right labels and model's prediction, to later compute accuracy
      labels.append(batch_out['label'])
      prediction.append(torch.argmax(batch_out['model_logits']))
    # Mean loss of batches
    loss = batches_loss / (len(batches))
    if log_stuff:
      self.log('val_loss', loss, prog_bar=True)
    return {'loss': loss, 'labels': labels, 'prediction': prediction}

  def validation_epoch_end(self, validation_step_outputs):
    if log_stuff:
      predictions = []
      labels = []
      # Merge all the outputs together
      for out in validation_step_outputs:
        predictions.extend(out['prediction'])
        labels.extend(out['labels'])
      self.log('val_acc', torchmetrics.Accuracy()(torch.tensor(predictions), torch.tensor(labels)), prog_bar=True)

  def test_step(self, batches, batch_id):
    batches_loss = 0
    prediction = []
    labels = []
    for batch in batches:
      batch_out = self.forward(*batch)
      batches_loss += batch_out['forward_loss']
      # Validating and testing I also extract the right labels and model's prediction, to later compute accuracy
      labels.append(batch_out['label'])
      prediction.append(torch.argmax(batch_out['model_logits']))
    # Mean loss of batches
    loss = batches_loss / (len(batches))
    if log_stuff:
      self.log('test_loss', loss, prog_bar=True)
    return {'loss': loss, 'labels': labels, 'prediction': prediction}

  def test_epoch_end(self, test_step_outputs):
    if log_stuff:
      predictions = []
      labels = []
      # Merge all the outputs together
      for out in test_step_outputs:
        predictions.extend(out['prediction'])
        labels.extend(out['labels'])
      self.log('test_accuracy', torchmetrics.Accuracy()(torch.tensor(predictions), torch.tensor(labels)), prog_bar=True)

  # ----------------------------------------------- PREDICT -----------------------------------------------
  def predict(self, sentence_pairs: List[Dict]) -> Tuple[List[str], List[str]]:
    # I cannot just feed sentence_pairs to the model because that's unprocessed data
    wsd_predictions = []
    wic_predictions = []
    for elem in sentence_pairs:
      # --- Generate sample1 ---
      instance1 = elem['sentence1'][int(elem['start1']):int(elem['end1'])].lower()
      sample1, keys = self.predict_generate_sample(elem['sentence1'].lower(), instance1, pos_map[elem['pos']], elem['lemma'])
      if len(keys) == 0:
        wsd_out1 = 'None'
      else:
        model_logits = torch.zeros(len(sample1), dtype=torch.double, device=Device)
        # --- Predict sample1 ---
        with torch.no_grad():
          # Input the sample in the model and store the output
          for i, item in enumerate(sample1):
            model_logits[i] = self.forward(*item, None)
        wsd_out1 = keys[torch.max(torch.softmax(model_logits, dim=0), -1).indices] # key with highest probability
      # --- Generate sample2 ---
      instance2 = elem['sentence2'][int(elem['start2']):int(elem['end2'])].lower()
      sample2, keys = self.predict_generate_sample(elem['sentence2'].lower(), instance2, pos_map[elem['pos']], elem['lemma'])
      if len(keys) == 0:
        wsd_out2 = 'None'
      else:
        model_logits = torch.zeros(len(sample2), dtype=torch.double, device=Device)
        # --- Predict sample2 ---
        with torch.no_grad():
          # Input the sample in the model and store the output
          for i, item in enumerate(sample2):
            model_logits[i] = self.forward(*item, None)
        wsd_out2 = keys[torch.max(torch.softmax(model_logits, dim=0), -1).indices] # key with highest probability
      # ------ Store predictions ------
      wsd_predictions.append((wsd_out1, wsd_out2))
      wic_predictions.append('True' if wsd_out1 == wsd_out2 else 'False')
    return wic_predictions, wsd_predictions

  # This function is used in the predict function. It builds a sample which items are feedable to the model.
  def predict_generate_sample(self, sentence, ambiguous_word, pos, lemma):
    # ---------------- Same as generating a sample in generate_samples function ---------------
    keys = []
    glosses = []
    sample = []
    # ------------------ 1: Find all the possible glosses and keys for this ambiguous word ------------------
    for synset in (wn.synsets(lemma.lower(), pos)):
      for _lemma in synset.lemmas():
        if _lemma.name().lower() == lemma.lower():
          key = _lemma.key()
          break
        elif _lemma.name().lower() in wn._morphy(lemma.lower(), pos):
          key = _lemma.key()
        else:
          key = ''
      keys.append(key)
      glosses.append(synset.definition())
    # ------------------ 2: Sentence ------------------
    # Find the index of the ambiguous word instance, add [TRG] tokens near it
    instance_position = sentence.find(ambiguous_word)
    if instance_position >= 0:
      preproc_sentence = sentence[0:instance_position] + '[TRG] ' + ambiguous_word + ' [TRG]' + sentence[instance_position+len(ambiguous_word):]
    else:
      preproc_sentence = sentence # just in case, but this should never happen
    # ---------------- 3: Here the two elements are merged together to form the sample -------------
    sentence_tokens = tokenizer.tokenize(preproc_sentence)
    for gloss in glosses:
      gloss_tokens = tokenizer.tokenize(gloss)
      ids_categories = [0] * (len(sentence_tokens) + 2) + [1] * (len(gloss_tokens) + 1)
      # From tokens to indexes..
      tokens = ['[CLS]'] + sentence_tokens + ['[SEP]'] + gloss_tokens + ['[SEP]']
      ids = tokenizer.convert_tokens_to_ids(tokens)
      attention_masks = [1] * len(ids)
      # ..To tensors in device
      ids = torch.tensor(ids).unsqueeze(0).to(Device)
      attention_masks = torch.tensor(attention_masks).unsqueeze(0).to(Device)
      ids_categories = torch.tensor(ids_categories).unsqueeze(0).to(Device)
      # Store in sample
      sample.append([ids, attention_masks, ids_categories]) # There are no labels inside the predict_samples
    return sample, keys
