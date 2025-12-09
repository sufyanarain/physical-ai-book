import os
import sys
import asyncio
from groq import AsyncGroq
from pathlib import Path
from dotenv import load_dotenv

# Fix Windows console encoding for emojis
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding='utf-8')

# Load environment variables
load_dotenv()

async def translate_to_urdu(client, content):
    """Use Groq LLM to translate content to Urdu."""
    try:
        response = await client.chat.completions.create(
            model="llama-3.3-70b-versatile",
            messages=[{
                "role": "system",
                "content": (
                    "You are a professional translator specializing in technical documentation. "
                    "Translate the following technical content from English to Urdu. "
                    "\n\n"
                    "IMPORTANT INSTRUCTIONS:\n"
                    "1. Maintain ALL markdown formatting exactly (headers, lists, code blocks, links, images)\n"
                    "2. Keep frontmatter (---) completely unchanged\n"
                    "3. Do NOT translate:\n"
                    "   - Code blocks (keep code in English)\n"
                    "   - Technical terms that are commonly used in English (ROS 2, Python, API, etc.)\n"
                    "   - URLs and links\n"
                    "   - File paths and commands\n"
                    "4. Translate:\n"
                    "   - All explanatory text\n"
                    "   - Headers and titles\n"
                    "   - Descriptions and examples\n"
                    "5. Use appropriate Urdu technical terminology where available\n"
                    "6. Maintain right-to-left (RTL) directionality for Urdu text\n"
                    "7. Keep the same document structure and formatting\n"
                )
            }, {
                "role": "user",
                "content": f"Translate this documentation to Urdu:\n\n{content}"
            }],
            temperature=0.3,
            max_tokens=8000
        )
        return response.choices[0].message.content
    except Exception as e:
        print(f"Error translating content: {e}")
        return content  # Return original on error

async def generate_urdu_docs():
    """Generate Urdu documentation from English source."""
    client = AsyncGroq(api_key=os.getenv("GROQ_API_KEY"))

    # Define paths
    source_dir = Path("../website/docs")
    urdu_dir = Path("../website/docs-urdu")

    print("🚀 Starting Urdu documentation generation...")
    print(f"Source: {source_dir.absolute()}")
    print(f"Urdu version: {urdu_dir.absolute()}\n")

    # Create directory
    urdu_dir.mkdir(exist_ok=True)

    # Find all markdown files
    md_files = list(source_dir.rglob("*.md"))
    total = len(md_files)
    print(f"📚 Found {total} markdown files to translate\n")

    for idx, md_file in enumerate(md_files, 1):
        try:
            content = md_file.read_text(encoding='utf-8')
            relative_path = md_file.relative_to(source_dir)

            print(f"[{idx}/{total}] Translating: {relative_path}")

            # Generate Urdu translation
            print(f"  → Translating to Urdu...")
            urdu_content = await translate_to_urdu(client, content)

            # Save file maintaining directory structure
            urdu_file = urdu_dir / relative_path
            urdu_file.parent.mkdir(parents=True, exist_ok=True)
            urdu_file.write_text(urdu_content, encoding='utf-8')

            print(f"  ✓ Urdu translation complete\n")

        except Exception as e:
            print(f"  ✗ Error processing {relative_path}: {e}\n")
            continue

    print("\n✅ Urdu documentation generation complete!")
    print(f"📁 Urdu docs: {urdu_dir.absolute()}")

if __name__ == "__main__":
    asyncio.run(generate_urdu_docs())
