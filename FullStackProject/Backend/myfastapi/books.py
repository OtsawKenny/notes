from fastapi import FastAPI, HTTPException

app = FastAPI()

BOOKS = [
    {'title': 'Title One', 'author': 'Author One', 'category': 'science'},
    {'title': 'Title Two', 'author': 'Author Two', 'category': 'science'},
    {'title': 'Title Three', 'author': 'Author Three', 'category': 'history'},
    {'title': 'Title Four', 'author': 'Author Four', 'category': 'math'},
    {'title': 'Title Five', 'author': 'Author Five', 'category': 'math'}
]

class Book:
    id: int
    title: str
    author: str
    description: str
    rating:int
    
    def __init__(self, id, title, author, description, rating):
        self.id = id
        self.title = title
        self.author = author
        self.description = description
        self.rating = rating

@app.get("/books")
async def get_all_books():
    return BOOKS

@app.get("/books/")
async def get_all_books_by_query(category: str):
    books_to_return = []
    for book in BOOKS:
        if book.get('category').casefold() == category.casefold():
            books_to_return.append(book)
    return books_to_return

@app.get("/books/{id}")
async def get_book(id: int):
    try:
        return BOOKS[id]
    except IndexError:
        raise HTTPException(status_code=404, detail="Book not found")

@app.post("/create-books")
async def get_all_books():
    return BOOKS
