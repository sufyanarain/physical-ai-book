"""
Script to populate the vector database with textbook content from local markdown files
"""
from app.vector_db import VectorDB
import os
import sys
from pathlib import Path

def read_markdown_files(docs_dir):
    """Read all markdown files from docs directory"""
    documents = []
    docs_path = Path(docs_dir)
    
    for md_file in docs_path.rglob("*.md"):
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
                
            # Extract title from first # heading or use filename
            title = md_file.stem
            lines = content.split('\n')
            for line in lines:
                if line.startswith('# '):
                    title = line.replace('# ', '').strip()
                    break

            # Improved chunking: split by ## headings to keep sections together
            sections = []
            current_section = []
            current_heading = title

            for line in lines:
                if line.startswith('## '):
                    # Save previous section if it has content
                    if current_section:
                        section_text = '\n'.join(current_section).strip()
                        if len(section_text) > 50:
                            sections.append({
                                "heading": current_heading,
                                "content": section_text
                            })
                    # Start new section
                    current_heading = line.replace('## ', '').strip()
                    current_section = [line]
                else:
                    current_section.append(line)

            # Don't forget the last section
            if current_section:
                section_text = '\n'.join(current_section).strip()
                if len(section_text) > 50:
                    sections.append({
                        "heading": current_heading,
                        "content": section_text
                    })

            # Create documents from sections
            for i, section in enumerate(sections):
                documents.append({
                    "title": f"{title} - {section['heading']}",
                    "content": section['content'],
                    "source": str(md_file.relative_to(docs_path.parent))
                })
        except Exception as e:
            print(f"Error reading {md_file}: {e}")
    
    return documents

def main():
    # Path to docs directory
    project_root = Path(__file__).parent.parent
    docs_dir = project_root / "website" / "docs"
    
    if not docs_dir.exists():
        print(f"ERROR: Docs directory not found: {docs_dir}")
        sys.exit(1)

    print(f"Reading markdown files from: {docs_dir}")
    documents = read_markdown_files(docs_dir)

    if not documents:
        print("ERROR: No content found")
        sys.exit(1)

    print(f"Found {len(documents)} text chunks from markdown files")

    print("\nSetting up vector database...")
    vector_db = VectorDB()
    vector_db.create_collection()

    print("\nAdding documents to vector database...")
    vector_db.add_documents(documents)

    print("\nIndexing complete!")
    print(f"   - Chunks indexed: {len(documents)}")
    print(f"   - Collection: {vector_db.collection_name}")

if __name__ == "__main__":
    main()
