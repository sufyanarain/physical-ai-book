"""
Database migration script to update user schema from two-dimensional experience to single background_type
"""
import os
from sqlalchemy import create_engine, text
from dotenv import load_dotenv

load_dotenv()

def migrate_database():
    """Migrate user table to use background_type instead of software/hardware experience"""
    
    db_url = os.getenv('NEON_DATABASE_URL') or os.getenv('DATABASE_URL')
    
    if not db_url:
        print("❌ No database URL found in environment variables")
        return
    
    engine = create_engine(db_url)
    
    print("🔄 Starting database migration...")
    
    try:
        with engine.connect() as conn:
            # Check if the new column exists
            check_column = text("""
                SELECT column_name 
                FROM information_schema.columns 
                WHERE table_name='users' AND column_name='background_type'
            """)
            result = conn.execute(check_column)
            
            if result.fetchone():
                print("✅ Migration already completed - background_type column exists")
                return
            
            # Add new column
            print("  → Adding background_type column...")
            conn.execute(text("""
                ALTER TABLE users 
                ADD COLUMN background_type VARCHAR(50) DEFAULT 'software'
            """))
            conn.commit()
            
            # Migrate existing data (users with advanced software experience = software background, otherwise hardware)
            print("  → Migrating existing user data...")
            conn.execute(text("""
                UPDATE users 
                SET background_type = CASE 
                    WHEN software_experience IN ('intermediate', 'advanced') THEN 'software'
                    ELSE 'hardware'
                END
            """))
            conn.commit()
            
            # Drop old columns (optional - comment out if you want to keep them for backup)
            print("  → Removing old experience columns...")
            conn.execute(text("""
                ALTER TABLE users 
                DROP COLUMN IF EXISTS software_experience,
                DROP COLUMN IF EXISTS hardware_experience
            """))
            conn.commit()
            
            print("✅ Database migration completed successfully!")
                
    except Exception as e:
        print(f"❌ Database connection failed: {e}")

if __name__ == "__main__":
    migrate_database()
