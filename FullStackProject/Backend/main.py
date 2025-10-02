from fastapi import FastAPI
import uvicorn

app = FastAPI()

@app.get("/")
async def root():
    return {"message": "Hello World"}

def main():
    # Start the FastAPI server with Uvicorn
    uvicorn.run(
        app,
        host="localhost",
        port=8000,
    )
    
if __name__ == "__main__":
    main()
    
    
# FMCS2.0 
# Robotic Fleet Management and Control System


 