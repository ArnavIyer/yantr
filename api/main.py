import logging

import uvicorn
from fastapi import FastAPI, UploadFile

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("api")

app = FastAPI(title="Yantr Vision API")


@app.get("/health")
async def health():
    return {"status": "ok"}


@app.post("/upload-image")
async def upload_image(file: UploadFile):
    contents = await file.read()
    size = len(contents)
    logger.info(
        "Received image: filename=%s content_type=%s size=%d bytes",
        file.filename,
        file.content_type,
        size,
    )
    return {
        "filename": file.filename,
        "content_type": file.content_type,
        "size_bytes": size,
    }


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
