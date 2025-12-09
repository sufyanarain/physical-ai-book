import os
import asyncio
from groq import AsyncGroq
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

async def adapt_for_background(client, content, background, instruction):
    """Use Groq LLM to adapt content for specific background."""
    try:
        response = await client.chat.completions.create(
            model="llama-3.3-70b-versatile",
            messages=[{
                "role": "system",
                "content": f"You are adapting technical documentation for users with {background} background. "
                          f"{instruction}\n\n"
                          f"IMPORTANT: Maintain ALL markdown formatting, frontmatter (---), links, images, and structure exactly. "
                          f"Only modify the explanatory text to add helpful context for the target audience."
            }, {
                "role": "user",
                "content": f"Adapt this documentation:\n\n{content}"
            }],
            temperature=0.3,
            max_tokens=8000
        )
        return response.choices[0].message.content
    except Exception as e:
        print(f"Error adapting content: {e}")
        return content  # Return original on error

async def generate_background_docs():
    """Generate separate documentation for software and hardware backgrounds."""
    client = AsyncGroq(api_key=os.getenv("GROQ_API_KEY"))
    
    # Define paths
    source_dir = Path("../website/docs")
    software_dir = Path("../website/docs-software")
    hardware_dir = Path("../website/docs-hardware")
    
    print("🚀 Starting documentation generation...")
    print(f"Source: {source_dir.absolute()}")
    print(f"Software version: {software_dir.absolute()}")
    print(f"Hardware version: {hardware_dir.absolute()}\n")
    
    # Create directories
    software_dir.mkdir(exist_ok=True)
    hardware_dir.mkdir(exist_ok=True)
    
    # Find all markdown files
    md_files = list(source_dir.rglob("*.md"))
    total = len(md_files)
    print(f"📚 Found {total} markdown files to process\n")
    
    for idx, md_file in enumerate(md_files, 1):
        try:
            content = md_file.read_text(encoding='utf-8')
            relative_path = md_file.relative_to(source_dir)
            
            print(f"[{idx}/{total}] Processing: {relative_path}")
            
            # Generate software background version
            print(f"  → Generating software background version...")
            software_content = await adapt_for_background(
                client, content, "software",
                "Add detailed explanations for hardware/electronics/robotics concepts. "
                "Assume user is proficient in programming (Python, OOP, algorithms) but needs help "
                "understanding circuits, sensors, actuators, motors, mechanical systems, and physical principles. "
                "Add practical analogies between software and hardware concepts where helpful."
            )
            
            # Generate hardware background version
            print(f"  → Generating hardware background version...")
            hardware_content = await adapt_for_background(
                client, content, "hardware",
                "Add detailed explanations for programming/software/CS concepts. "
                "Assume user is proficient in electronics and mechanics but needs help with "
                "code structure, algorithms, data structures, OOP, Python syntax, and software design patterns. "
                "Add practical analogies between hardware and software concepts where helpful."
            )
            
            # Save files maintaining directory structure
            software_file = software_dir / relative_path
            hardware_file = hardware_dir / relative_path
            
            software_file.parent.mkdir(parents=True, exist_ok=True)
            hardware_file.parent.mkdir(parents=True, exist_ok=True)
            
            software_file.write_text(software_content, encoding='utf-8')
            hardware_file.write_text(hardware_content, encoding='utf-8')
            
            print(f"  ✓ Generated both versions\n")
            
        except Exception as e:
            print(f"  ✗ Error processing {relative_path}: {e}\n")
            continue
    
    print("\n✅ Documentation generation complete!")
    print(f"📁 Software background docs: {software_dir.absolute()}")
    print(f"📁 Hardware background docs: {hardware_dir.absolute()}")

if __name__ == "__main__":
    asyncio.run(generate_background_docs())
