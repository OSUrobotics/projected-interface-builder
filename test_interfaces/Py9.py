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

class Predictor(object):
    seq = []
    def __init__(self, lettermap, words='/usr/share/dict/words'):
        self.lettermap = lettermap
        with open(words,'r') as f:
            self.words = f.readlines()
    def seq_to_letters(self, seq):
        return [self.lettermap[n] for n in seq]
            
    def predict(self, seq):
        words = self.words
        for i, letters in enumerate(self.seq_to_letters(seq)):
            words = filter(lambda w: w[i].upper() in letters, words)
        return [w.strip() for w in words]

    def predict_incremental(self, num):
        self.seq.append(num)
        return self.predict(self.seq)