from database import qdrant_client, cohere_client, llm_client

MODEL = "google/gemini-2.0-flash-001"

def get_context(query, selected_text=None):
    context_parts = []
    
    # 1. Add Highlighted text if valid
    if selected_text and selected_text.strip() and selected_text.lower() != "string":
        context_parts.append(f"USER HIGHLIGHTED TEXT: {selected_text}")

    # 2. Retrieve from Qdrant
    try:
        emb_res = cohere_client.embed(
            texts=[query], 
            model="embed-english-v3.0", 
            input_type="search_query"
        )
        q_emb = emb_res.embeddings.float_[0] 
        
        search_result = qdrant_client.query_points(
            collection_name="book_docs",
            query=q_emb,
            limit=10
        )
        
        docs = [hit.payload["text"] for hit in search_result.points if hit.payload and "text" in hit.payload]

        if docs:
            rerank = cohere_client.rerank(
                model="rerank-english-v3.0", 
                query=query, 
                documents=docs, 
                top_n=3
            )
            qdrant_context = "\n\n".join([docs[r.index] for r in rerank.results])
            context_parts.append(f"BOOK PASSAGES FROM DATABASE: {qdrant_context}")
            
    except Exception as e:
        print(f"Retrieval error: {e}")

    return "\n\n".join(context_parts) if context_parts else None

async def ask_bot(query, selected_text=None):
    try:
        context = get_context(query, selected_text)
        
        # If no context was found at all, we can stop early or let the LLM handle it
        if not context:
            return "I am sorry, but I don't know the answer to that as I couldn't find any relevant information in the book."

        response = llm_client.chat.completions.create(
            model=MODEL,
            messages=[
                {
                    "role": "system", 
                    "content": (
                        "You are a strict Book Assistant. Your ONLY source of truth is the provided context. "
                        "1. If the answer is NOT in the context, say: 'I am sorry, but I don't know the answer to that as it is not mentioned in the book.' "
                        "2. Do NOT use any outside knowledge or talk about real-world events not in the text. "
                        "3. Do NOT make up facts. If the context is even slightly insufficient, admit you don't know."
                    )
                },
                {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {query}"}
            ],
            temperature=0.1 # Lower temperature makes the bot more factual and less creative
        )
        return response.choices[0].message.content
    except Exception as e:
        return f"Backend Error: {str(e)}"