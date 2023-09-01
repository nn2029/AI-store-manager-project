from collections import defaultdict

class ProductDebounce:
    """
    ProductDebounce class is used to ensure that a product detection is consistent over several frames.
    This prevents erroneous triggers and ensures that a product is genuinely present before counting it.

    Attributes:
    -----------
    size : int
        The number of frames a product needs to be consistently detected to be considered valid.
    history : defaultdict(list)
        A dictionary mapping product IDs to a list of their detection history over the last `size` frames.
    """

    def __init__(self, size=3):
        """
        Initialize the ProductDebounce with the given size.

        Parameters:
        -----------
        size : int
            The number of frames a product needs to be consistently detected to be considered valid.
        """
        self.size = size
        self.history = defaultdict(lambda: [])  

    def add_observation(self, product_id):
        """
        Add a detection observation for a given product.

        Parameters:
        -----------
        product_id : int
            The ID of the detected product.
        """
        if len(self.history[product_id]) >= self.size:
            self.history[product_id].pop(0)
        self.history[product_id].append(True)

    def should_count(self, product_id):
        """
        Determine if a product should be counted based on its detection history.

        Parameters:
        -----------
        product_id : int
            The ID of the detected product.

        Returns:
        --------
        bool
            True if the product should be counted, False otherwise.
        """
        return len(self.history[product_id]) == self.size and all(self.history[product_id])
