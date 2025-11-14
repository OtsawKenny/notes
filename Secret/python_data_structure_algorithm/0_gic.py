print("HELLO")

[]
{"name":"teo", "floor_request":([1,1],[4,4]),"floor_allocated":([],[]) , "floor_share":([],[])}
{"name":"aliyah", "floor_request":([3,3],[5,5]),"floor_allocated":([],[]) , "floor_share":([],[])}



map = []

def init_map(x,y):
    for i in range ( 0 , x ):
        map.append([])
        for j in range ( 0 , y):
            map[i].append(0)
            

def print_map():
    print("==Visulise the floor area ==")
    for i in map:
        print(i)    
        
def allocate_map(id,tuple): 
    x1,y1=tuple[0]
    x2,y2=tuple[1]
    for i in range (x1,x2+1):
        for j in range (y1,y2+1):
            if(map[i][j]==0):
                #Free Space
                map[i][j]=id
            else:
                #Contested Space
                map[i][j]=-1


def count_area():
    dict_area = {}
    dict_area[0]=0
    dict_area[-1]=0
    for i in map_request:
        dict_area[i]=0
    print('dict_area ',dict_area)
    for i in range(0,len(map)):
        for j in range (0,len(map[0])):
            # print(i,j)
            # print(map[i][j])
            dict_area[map[i][j]] +=1
    
    print('dict_area', dict_area)

map_request= {
    "t":[(1,1),(3,3)],
    "a":[(4,0),(4,5)],
    "w":[(2,2),(5,5)],
}

init_map(10,10)
print_map()

print("\n")
print("==Allocation floor area ==")
for id in map_request:
    allocate_map(id, map_request[id])
    print_map()

print("==Count floor area ==")
count_area()