#!/usr/bin/env python
import nltk
from nltk.corpus import brown
import collections
import pickle
import sys
if __name__ == '__main__':
    bgm = nltk.collocations.BigramAssocMeasures()
    finder = nltk.collocations.BigramCollocationFinder.from_words(brown.words())
    scored = finder.score_ngrams(bgm.likelihood_ratio)

    score_dict = collections.defaultdict(dict)
    for key, score in scored:
        score_dict[key[0].lower()][key[1].lower()] = score
    
    with open(sys.argv[1],'w') as f:
        pickle.dump(score_dict, f)
    print 'wrote ', f.name