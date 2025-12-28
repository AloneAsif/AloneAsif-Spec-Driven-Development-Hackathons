import requests
from bs4 import BeautifulSoup
from database import qdrant_client, cohere_client
from qdrant_client.models import PointStruct, VectorParams, Distance

# Configuration
SITEMAP_URL = "https://alone-asif-spec-driven-development.vercel.app/sitemap.xml"
# This is the domain we WANT to index. 
# We will replace any placeholder domains found in the sitemap with this.
REAL_DOMAIN = "https://alone-asif-spec-driven-development.vercel.app"

def ingest():
    # 1. Setup Qdrant Collection
    collection_name = "book_docs"
    if not qdrant_client.collection_exists(collection_name):
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
        )

    # 2. Fetch and Parse Sitemap XML manually
    print(f"📡 Fetching sitemap: {SITEMAP_URL}")
    response = requests.get(SITEMAP_URL)
    if response.status_code != 200:
        print(f"❌ Could not find sitemap at {SITEMAP_URL}. Status: {response.status_code}")
        return

    # Use 'xml' parser to find all <loc> tags
    soup = BeautifulSoup(response.content, "xml")
    raw_urls = [loc.text.strip() for loc in soup.find_all("loc")]

    if not raw_urls:
        print("❌ No URLs found inside the sitemap XML.")
        return

    # 3. Clean and Filter URLs
    # This fixes the issue where the sitemap might contain "your-docusaurus-site.example.com"
    processed_urls = []
    for url in raw_urls:
        if "example.com" in url:
            # Swap placeholder with your real Vercel URL
            clean_url = url.replace("https://your-docusaurus-site.example.com", REAL_DOMAIN)
            processed_urls.append(clean_url)
        else:
            processed_urls.append(url)

    print(f"✅ Found {len(processed_urls)} pages to index.")

    # 4. Scrape and Embed
    for i, url in enumerate(processed_urls):
        try:
            print(f"📖 Processing: {url}")
            page_res = requests.get(url, timeout=10)
            page_soup = BeautifulSoup(page_res.text, "html.parser")
            
            # Extract clean text
            for garbage in page_soup(["script", "style", "nav", "footer"]):
                garbage.decompose()
            text = page_soup.get_text(" ", strip=True)[:3500] 

            if len(text) < 100: continue

            # Get Embedding from Cohere
            emb = cohere_client.embed(
                texts=[text], model="embed-english-v3.0", input_type="search_document"
            ).embeddings.float_[0]

            qdrant_client.upsert(
                collection_name=collection_name,
                points=[PointStruct(id=i, vector=emb, payload={"text": text, "url": url})]
            )
        except Exception as e:
            print(f"⚠️ Error on {url}: {e}")

    print("\n✨ All done! Your chatbot now has the book's knowledge.")

if __name__ == "__main__":
    ingest()