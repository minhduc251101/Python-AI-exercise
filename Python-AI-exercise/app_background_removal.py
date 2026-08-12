from fastapi import FastAPI, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from rembg import remove
import io

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # đổi thành origin FE khi deploy
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.post("/remove-bg")
async def remove_bg(file: UploadFile = File(...)):
    input_bytes = await file.read()
    output_bytes = remove(input_bytes, force_return_bytes=True)

    return StreamingResponse(
        io.BytesIO(output_bytes),
        media_type="image/png",
        headers={"Content-Disposition": "inline; filename=output.png"},
    )
