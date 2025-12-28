import os
from openai import OpenAI
from qdrant_client import QdrantClient
from cohere import ClientV2 as CohereClient
from dotenv import load_dotenv

load_dotenv()

# OpenRouter Client (OpenAI SDK Compatible)
llm_client = OpenAI(
    base_url="https://openrouter.ai/api/v1",
    api_key=os.getenv("OPENROUTER_API_KEY"),
    default_headers={
        "HTTP-Referer": "http://localhost:8000", # Required by OpenRouter
        "X-Title": "Book RAG Bot"
    }
)

# Cohere Client for Embeddings & Reranking
cohere_client = CohereClient(api_key=os.getenv("COHERE_API_KEY"))

# Qdrant Vector Store
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"), 
    api_key=os.getenv("QDRANT_API_KEY")
)