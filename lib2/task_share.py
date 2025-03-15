"""!
@file task_share.py
@brief Provides classes for data sharing and communication between tasks.
@details
  Includes two main classes: Queue (for multiple data items) and Share (for single data).
  Also contains a global function show_all() which prints diagnostics for all shares/queues.

@author
   JR Ridgely

@date
   2017-Jan-01 (original creation), 2021-Dec-18 (docstring updates),
   2023 (further modifications and reformatting)

@copyright
   Released under the GNU Public License, version 3.0.
   Intended for educational use only; no warranties implied.
"""

import array
import gc
import pyb
import micropython

## @brief A system-wide list of all queues and shared variables for diagnostics.
share_list = []

## @brief Maps Python array type codes to human-readable strings.
type_code_strings = {
    'b': "int8",   'B': "uint8",
    'h': "int16",  'H': "uint16",
    'i': "int(?)", 'I': "uint(?)",
    'l': "int32",  'L': "uint32",
    'q': "int64",  'Q': "uint64",
    'f': "float",  'd': "double"
}

def show_all():
    """!
    @brief Generates a diagnostic printout of all queues and shares.
    @return A string containing formatted data about each queue/share.
    """
    gen = (str(item) for item in share_list)
    return '\n'.join(gen)


class BaseShare:
    """!
    @brief Base class for data-sharing structures (Queue & Share).
    @details
      Not to be instantiated directly. Provides common functionality for 
      child classes.
    """

    def __init__(self, type_code, thread_protect=True, name=None):
        """!
        @brief Initializes common features of share/queue objects.
        @param type_code One-letter type code as for array.array (e.g. 'f' for float).
        @param thread_protect If True, disables interrupts during data access.
        @param name Optional name string for diagnostic identification.
        """
        self._type_code = type_code
        self._thread_protect = thread_protect
        share_list.append(self)


class Queue(BaseShare):
    """!
    @brief A FIFO queue to transfer data between tasks.
    @details
      Stores items of a single type. Can be configured to overwrite old data or block 
      if full, depending on constructor parameters.
    """

    ser_num = 0  # Counter for naming unnamed queues

    def __init__(self, type_code, size, thread_protect=False, overwrite=False, name=None):
        """!
        @brief Initializes the queue with the given size and type.
        @param type_code Data type (one-letter code, e.g. 'h', 'f', etc.).
        @param size Maximum number of items the queue can hold.
        @param thread_protect If True, disable interrupts during put/get operations.
        @param overwrite If True, new data overwrites old when full instead of blocking.
        @param name Optional name for this queue.
        """
        super().__init__(type_code, thread_protect, name)
        self._size = size
        self._overwrite = overwrite
        self._name = str(name) if name else ('Queue' + str(Queue.ser_num))
        Queue.ser_num += 1

        try:
            self._buffer = array.array(type_code, range(size))
        except MemoryError:
            self._buffer = None
            raise
        except ValueError:
            self._buffer = None
            raise

        self.clear()
        gc.collect()

    @micropython.native
    def put(self, item, in_ISR=False):
        """!
        @brief Puts an item into the queue. Blocks if full (unless overwrite=True).
        @param item The data item to store.
        @param in_ISR True if called from an ISR, in which case it cannot block.
        """
        if self.full():
            if in_ISR:
                return
            if not self._overwrite:
                while self.full():
                    pass

        if self._thread_protect and not in_ISR:
            _irq_state = pyb.disable_irq()

        self._buffer[self._wr_idx] = item
        self._wr_idx += 1
        if self._wr_idx >= self._size:
            self._wr_idx = 0
        self._num_items += 1
        if self._num_items >= self._size:
            self._num_items = self._size
        if self._num_items > self._max_full:
            self._max_full = self._num_items

        if self._thread_protect and not in_ISR:
            pyb.enable_irq(_irq_state)

    @micropython.native
    def get(self, in_ISR=False):
        """!
        @brief Retrieves an item from the queue. Blocks if empty (unless in_ISR=True).
        @param in_ISR True if called from an ISR, in which case it cannot block.
        @return The data item read from the queue.
        """
        while self.empty():
            if in_ISR:
                return None

        if self._thread_protect and not in_ISR:
            irq_state = pyb.disable_irq()

        to_return = self._buffer[self._rd_idx]
        self._rd_idx += 1
        if self._rd_idx >= self._size:
            self._rd_idx = 0
        self._num_items -= 1
        if self._num_items < 0:
            self._num_items = 0

        if self._thread_protect and not in_ISR:
            pyb.enable_irq(irq_state)

        return to_return

    @micropython.native
    def any(self):
        """!
        @brief Checks if the queue has at least one item.
        @return True if there is data, False if empty.
        """
        return (self._num_items > 0)

    @micropython.native
    def empty(self):
        """!
        @brief Checks if the queue is empty.
        @return True if no items in the queue, False otherwise.
        """
        return (self._num_items <= 0)

    @micropython.native
    def full(self):
        """!
        @brief Checks if the queue is full.
        @return True if the queue has reached its capacity, False otherwise.
        """
        return (self._num_items >= self._size)

    @micropython.native
    def num_in(self):
        """!
        @brief Returns the current number of items in the queue.
        @return The count of items in the buffer.
        """
        return (self._num_items)

    def clear(self):
        """!
        @brief Empties the queue of all data.
        """
        self._rd_idx = 0
        self._wr_idx = 0
        self._num_items = 0
        self._max_full = 0

    def __repr__(self):
        """!
        @brief Returns a string with diagnostic information about this queue.
        """
        return ('{:<12s} Queue<{:s}> Max Full {:d}/{:d}'.format(
                self._name, type_code_strings[self._type_code],
                self._max_full, self._size))


class Share(BaseShare):
    """!
    @brief A single shared data item (thread-safe if needed).
    @details
      Holds exactly one data item of a fixed type. Overwrites old data with each put().
    """

    ser_num = 0  # Counter for naming unnamed shares

    def __init__(self, type_code, thread_protect=True, name=None):
        """!
        @brief Creates a shared data item of a specific type.
        @param type_code Data type code (e.g. 'f', 'h', etc.).
        @param thread_protect Protect data by disabling interrupts if True.
        @param name Optional name for diagnostic identification.
        """
        super().__init__(type_code, thread_protect, name)
        self._buffer = array.array(type_code, [0])
        self._name = str(name) if name else ('Share' + str(Share.ser_num))
        Share.ser_num += 1

    @micropython.native
    def put(self, data, in_ISR=False):
        """!
        @brief Writes data into the share (overwrites old data).
        @param data The new data value to store.
        @param in_ISR True if called from an ISR, so we do not block.
        """
        if self._thread_protect and not in_ISR:
            irq_state = pyb.disable_irq()
        self._buffer[0] = data
        if self._thread_protect and not in_ISR:
            pyb.enable_irq(irq_state)

    @micropython.native
    def get(self, in_ISR=False):
        """!
        @brief Reads the data from the share.
        @param in_ISR True if called from an ISR, so we do not block.
        @return The data in the share.
        """
        if self._thread_protect and not in_ISR:
            irq_state = pyb.disable_irq()
        to_return = self._buffer[0]
        if self._thread_protect and not in_ISR:
            pyb.enable_irq(irq_state)
        return to_return

    def __repr__(self):
        """!
        @brief Returns a string with diagnostic information about this share.
        """
        return ("{:<12s} Share<{:s}>".format(self._name,
                type_code_strings[self._type_code]))
