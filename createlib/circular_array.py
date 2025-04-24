
class CircularArray():

    def __init__(self, size):
        # Initalize the size and set default values with reset method
        self.size = size
        self.reset()
    

    def reset(self):
        # Reset or initalize all values to default
        self.queue = [0.0] * self.size      # Holds the items
        self.count = 0                      # Number of enqueued items
        self.head = 0                       # Front of line
        self.tail = 0                       # Back of the line
    
    
    def enqueue(self, value):
        """Enqueues a given value to the tail of the queue."""
        # Enqueue given value to the tail
        self.queue[self.tail] = value

        # Update tail
        self.tail = (self.tail + 1) % self.size
        
        if self.count == self.size:
            # Array is full, overwriting values
            self.head = (self.head + 1) % self.size
        else:
            self.count += 1

    
    def dequeue(self):
        """Dequeues item at the head."""

        if self.count == 0:
            # Queue is empty
            return None
        
        # Dequeue item at head and set that value to 0.0
        value = self.queue[self.head]
        self.queue[self.head] = 0.0

        # Reset head and count values
        self.head = (self.head + 1) % self.size
        self.count -= 1

        # Return dequeued head
        return value
    
    
    def sum(self):
        """Sums the enqueued values."""
        total = 0

        # Sum all used values (adding unused values will do nothing)
        for i in range(self.size):
            total += (self.queue[i])

        return total


    def print(self):
        """Prints the enqueued values."""
        for i in range(self.count):
            # Get current used index and print it without a newline
            index = (self.head + i) % self.size
            print(self.queue[index], end=" ")
        
        # Print newline
        print()


    def get_current(self):
        """Returns most recent enqueued item."""
        prev = (self.tail - 1) % self.size
        return self.queue[prev]
    

    def get_prev(self):
        """Returns value before most recent enqueued item."""
        prev = (self.tail - 2) % self.size
        return self.queue[prev]


    def __str__(self):
        """Prints entire queue."""
        return str(self.queue)
