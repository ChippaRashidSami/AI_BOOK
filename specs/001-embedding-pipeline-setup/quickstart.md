# Quickstart Guide: Embedding Pipeline

## Prerequisites

- Python 3.11+
- uv package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Install dependencies using uv:
   ```bash
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   uv pip install -r requirements.txt
   ```

3. Create a `.env` file based on `.env.example`:
   ```bash
   cp .env.example .env
   ```

4. Update `.env` with your API keys:
   ```bash
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_HOST=your_qdrant_cloud_endpoint
   ```

## Usage

### Process a Docusaurus-based GitHub Pages site:

```bash
python src/main.py --url https://your-book.github.io
```

### Process with custom parameters:

```bash
python src/main.py --url https://your-book.github.io --chunk-size 500 --collection-name my-book-embeddings
```

## Configuration

- `--url`: The GitHub Pages URL containing the Docusaurus-based book
- `--chunk-size`: Size of text chunks for embedding (default: 1000 characters)
- `--collection-name`: Name of the Qdrant collection to store embeddings (default: 'rag_embeddings')
- `--batch-size`: Number of chunks to process in each batch (default: 10)

## Example

To process the example book at the deploy link:

```bash
python src/main.py --url https://hackathon-l4pp.vercel.app/
```

## Output

The pipeline will:
1. Crawl the provided URL to discover all pages
2. Extract text content from each page
3. Chunk the content to fit within token limits
4. Generate embeddings using Cohere
5. Store the embeddings in Qdrant with metadata
6. Provide a summary of the processing results