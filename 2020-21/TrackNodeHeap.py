class TrackNodeHeap:
    """
    Class to maintain min or max heaps for TrackNodes
    """
	# _isMaxHeap = indicator if the heap is min or max
	# _elementList = list of elements of the heap
	# _size = number of elements in the heap
	# _map = mapping of elements to ids in the heap.

    def __init__(self,isMax):
        """
        Initializes an empty max-TrackNodeHeap if isMax is true.
        Initializes an empty min-TrackNodeHeap if isMax is false.

        Parameter isMax: whether the heap is a max heap or min heap
        Precondition: isMax is a bool
        """
        assert type(isMax) == bool
        self._isMaxHeap= isMax
        self._elementList= []
        self._size = 0
        self._map = {}


    class Element():
        """
        An object of class Element houses a TrackNode and its priority in the
        TrackNodeHeap
        """
        #_node the Track node
        #_priority the priority in the heap of the Tracknode

        @property
        def node(self):
            return self._node

        @property
        def priority(self):
            return self._priority

        @priority.setter
        def priority(self, p):
            self._priority=p

        def __init__(self, node, priority):
            """
            Initializes an Element of TrackNodeHeap storing the node and priority
            """
            self._node= node
            self._priority= priority


        def __str__(self):
            """
            Overrides python function str(TrackNodeHeap.Element)
            """
            return "(" + str(self._node) + ", " + str(self._priority) + ")"


        def __eq__(self,ob):
            """
            Overrides "==" for elements
            """
            if ob is None or not isinstance(ob, TrackNodeHeap.Element):
                return False
            return self._node == ob.node and self._priority == ob.priority


    def add(self,node, priority):
        """
        Add a node with the respective priority the heap.
        Expected: O(log(size))
        Worst-case: O(size)

        Parameter node: the element to be added in the heap
        Precondition: node is not in the heap

        Parameter priority: the priority of node
        Precondition: priority is a float
        """
        if (node in self._map.keys()):
            raise Exception("node is already in the heap")
        self._map[node]= self._size
        self._elementList.append(self.Element(node, priority));
        self._size+= 1;
        self._bubbleUp(self._size - 1);

    def __len__(self):
        """
        Overrides python function "len(TrackNodeHeap)"
        """
        return self._size;

    def _swap(self, h, k):
        """
        Helper method to maintain class invariant
        """
        assert 0<=h<self._size and 0<=k<self._size;
        temph= self._elementList[h]
        tempk= self._elementList[k]
        self._elementList[h]= tempk
        self._elementList[k]= temph
        self._map[self._elementList[h].node]= h
        self._map[self._elementList[k].node]= k


    def _compareTo(self,p1, p2):
        """
        Helper method to maintain class invariant
        """
        if p1 == p2:
            return 0
        if self._isMaxHeap:
            return 1 if p1 > p2 else -1
        return 1 if p1 < p2 else -1;

    def _compareToIndex(self,h,k):
        """
        Helper method to maintain class invariant
        """
        return self._compareTo(self._elementList[h].priority, self._elementList[k].priority)

    def _bubbleUp(self,h):
        """
        Helper method to maintain class invariant
        """
        if h >= self._size:
            return
        # Invariant: 0 <= h < size and<br>
        #.......... The class invariant is true, except perhaps<br>
        #.......... that b[h] belongs above its parent (if h > 0) in the heap, not below it.
        while h > 0:
            p= (h - 1) // 2; # p is h's parent
            if self._compareToIndex(h, p) <= 0:
                return
            self._swap(h, p)
            h= p

    def peek(self):
        """
        Return node value with lowest priority if heap in min.
        Return node value with highest priority if heap in max.

        Time: O(1)

        If heap is empty, raises exception.
        """
        if self._size <= 0:
            raise Exception("heap is empty")
        return self._elementList[0].node

    def _bubbleDown(self, h):
        """
        Helper method to maintain class invariant
        """
        if h < 0 or self._size <= h:
            return
        k= 2*h + 1
        # Invariant: Class invariant is true except perhaps that
        # .......... b[h] belongs below one or both of its children and
        # .......... k is h's left child.
        while k < self._size: # while b[h] has a child
            uc= k if (k + 1 == self._size or self._compareToIndex(k, k + 1) >= 0) else k + 1 # uc is the bigger child
            if self._compareToIndex(h, uc) >= 0:
                return;
            self._swap(h, uc)
            h= uc
            k= 2*h + 1

    def poll(self):
        """
        Return and remove node value with lowest priority if heap in min.
        Return and remove node value with highest priority if heap in max.

        Expected time: O(log(size))
        Worst-case time: O(size)

        If heap is empty, raises exception.
        """
        if (self._size <= 0):
            raise Exception("heap is empty")
        node= self._elementList[0].node
        self._swap(0, self._size - 1)
        del self._elementList[self._size - 1]
        del self._map[node]
        self._size-= 1
        self._bubbleDown(0)
        return node

    def updatePriority(self, node, priority):
        """
        Change the priority of value of a node in the heap.
        Expected: O(log(size))
        Worst-case: O(size)

        Parameter node: the element whose priority needs updating
        Precondition: node is in the heap

        Parameter priority: the new priority of node
        Precondition: priority is a float
        """
        index= self._map[node]
        oldPriority= self._elementList[index].priority
        self._elementList[index].priority= priority
        t= self._compareTo(priority, oldPriority)
        if t == 0:
            return
        elif t < 0:
            self._bubbleDown(index)
        else:
            self._bubbleUp(index)
