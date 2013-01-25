#!/usr/bin/env python
nummap = {
    7 : 'ABC',
    8 : 'DEF',
    9 : 'GHI',
    4 : 'JKL',
    6 : 'MNO',
    1 : 'PQR',
    2 : 'STUV',
    3 : 'WXYZ'
}

phonemap = {
    1: '1',
    2: 'ABC2',
    3: 'DEF3',
    4: 'GHI4',
    5: 'JKL5',
    6: 'MNO6',
    7: 'PQRS7',
    8: 'TUV8',
    9: 'WXYZ9'
}

import cPickle

class Predictor(object):
    seq = []
    sent = ['.']
    def __init__(self, lettermap, prune=False, words='/usr/share/dict/words'):
        self.prune = prune
        self.lettermap = lettermap
        with open(words,'r') as f:
            self.words = f.readlines()
        
        with open('bigrams.pkl', 'r') as bigr_file:
            self.bigrams = cPickle.load(bigr_file)
            
    def word_score(self, word):
        scores = self.bigrams[self.sent[-1]]
        return scores[word] if word in scores else 0.0
        
        
    def seq_to_letters(self, seq):
        return [self.lettermap[n] for n in seq]
            
    def predict(self, seq):
        words = self.words
        for i, letters in enumerate(self.seq_to_letters(seq)):
            words = filter(lambda w: w[i].upper() in letters.upper(), words)
        words = sorted([w.strip() for w in words], key=self.word_score, reverse=True)
        if self.prune:
            words = filter(lambda w: self.word_score(w) > 0, words)
        return words

    def predict_incremental(self, num):
        self.seq.append(num)
        return self.predict(self.seq)
        
    def choose_word(self, word):
        self.sent.append(word)
        self.seq = []

    def delete_last_word(self):
        if len(self.sent) > 0:
            del self.sent[-1]

    def delete_last_letter(self):
        if len(self.seq) > 0:
            del self.seq[-1]
        
if __name__ == '__main__':
    pred = Predictor(nummap)
    how = [9,6,3]
    are = [7,1,8]
    you = [3,6,2]
    today = [2,6,8,7,3]
    sent = [how,are,you,today]
    from pprint import pprint
    pred_sent = []
    for word in sent:
        options = pred.predict(word)
        pprint(list(enumerate(options)))
        choice = int(raw_input('-->'))
        pred.choose_word(options[choice])
        pred_sent.append(options[choice])
        
    print ' '.join(pred_sent)
        