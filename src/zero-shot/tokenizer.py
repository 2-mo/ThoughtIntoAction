"""Public Big Vision tokenizer and PI05 prompt format used by OpenPI upstream."""
import hashlib
from pathlib import Path
import urllib.request

TOKENIZER_URL = 'https://storage.googleapis.com/big_vision/paligemma_tokenizer.model'
TOKENIZER_SHA256 = '8986bb4f423f07f8c7f70d0dbe3526fb2316056c17bae71b1ea975e77a168fc6'
DEFAULT_PATH = Path(__file__).resolve().parents[2] / 'outputs/zero-shot-assets/paligemma_tokenizer.model'


def verify(path):
    if hashlib.sha256(path.read_bytes()).hexdigest() != TOKENIZER_SHA256:
        raise ValueError(f'PaliGemma tokenizer checksum mismatch: {path}')


def prepare():
    if not DEFAULT_PATH.exists():
        DEFAULT_PATH.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(TOKENIZER_URL, timeout=60) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != TOKENIZER_SHA256:
            raise ValueError('Downloaded tokenizer checksum mismatch')
        DEFAULT_PATH.write_bytes(data)
    verify(DEFAULT_PATH)
    print(DEFAULT_PATH)


def tokenize(tokenizer, prompt, state, max_length):
    import numpy as np
    cleaned = prompt.strip().replace('_', ' ').replace('\n', ' ')
    bins = np.digitize(state, bins=np.linspace(-1, 1, 257)[:-1]) - 1
    text = f"Task: {cleaned}, State: {' '.join(map(str, bins))};\nAction: "
    tokens = tokenizer.encode(text, add_bos=True)
    if len(tokens) > max_length:
        raise ValueError(f'Prompt needs {len(tokens)} tokens, maximum is {max_length}; shorten --prompt')
    mask = [True] * len(tokens) + [False] * (max_length - len(tokens))
    return tokens + [0] * (max_length - len(tokens)), mask


if __name__ == '__main__':
    prepare()
