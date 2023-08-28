from collections import defaultdict

class ProductDebounce:
    def __init__(self, size=10):
        self.size = size
        self.history = defaultdict(lambda: [])  
    def add_observation(self, product_id):
        if len(self.history[product_id]) >= self.size:
            self.history[product_id].pop(0)
        self.history[product_id].append(True)

    def should_count(self, product_id):
        return len(self.history[product_id]) == self.size and all(self.history[product_id])
