# First DataStructure , Linked Lists
class LinkedList:
    def __init__(self, value: int) -> None:
        # Create new Node
        new_node = Node(value)
        self.head = new_node
        self.tail = new_node
        self.length = 1
        return

    def append(self, value: int) -> None:
        # Create new Node
        # Add Node to end of Linked List        
        
        return

    def prepend(self, value: int) -> None:
        # Create new Node
        # Add Node to beginning of Linked List
        return

    def insert(self, index: int, value: int):
        # Create new Node
        # insert Node
        return
    
class Node:
    def __init__(self, value: int) -> None:
        self.value : int = value
        self.next: Node = None
        return


my_linked_list = LinkedList(4)
print("head:", my_linked_list.head.value)
print("tail:", my_linked_list.tail.value)

