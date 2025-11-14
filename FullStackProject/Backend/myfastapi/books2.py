from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from pydantic import BaseModel, Field

app = FastAPI()


class Book(BaseModel):
    id: int
    title: str
    author: str
    description: str
    rating: int


class BookRequest(BaseModel):
    id: int
    title: str = Field(min_length=3)
    author: str = Field(min_length=1)
    description: str = Field(min_length=1, max_length=100)
    rating: int = Field(gt=0, lt=6)


BOOKS = [
    Book(id=1, title='Title One', author='Author One',
         description='Description One', rating=4),
    Book(id=2, title='Title One', author='Author One',
         description='Description One', rating=4),
    Book(id=3, title='Title One', author='Author One',
         description='Description One', rating=4),
]


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


@app.post("/books/")
async def create_books(book_request: BookRequest):
    book = Book(**book_request.model_dump())
    BOOKS.append(book)
    return book

def find_book_id(book: Book):
    if len(BOOKS) > 0:
        book.id = BOOKS[-1].id + 1
    else:
        book.id = 1
    return book

@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request: Request, exc: RequestValidationError):
    body = await request.body()
    print(f"[ERROR] Validation failed! details={exc.errors()} body={exc.body}")
    return JSONResponse(
        status_code=422,
        content={"detail": exc.errors(), "body": exc.body},
    )
