import pypdf
import sys

def extract_pages(pdf_path, start_page, end_page):
    try:
        reader = pypdf.PdfReader(pdf_path)
        text = ""
        # pypdf pages are 0-indexed, but book pages are 1-indexed.
        # TOC said "Chapter 1 ... (Page 24)".
        # Let's assume the physical page 24 corresponds to index 23 usually, but PDF page numbers can differ.
        # I'll just extract based on 0-index. If the book says page 24, I'll try index 23-28.
        # But wait, the TOC page numbers from pypdf `get_destination_page_number` were 0-indexed assignments in my previous script? 
        # Yes, I added +1 for display. So "Page 24" in my TOC output means index 23.
        
        start_idx = start_page - 1
        end_idx = end_page
        
        print(f"Extracting pages {start_page} to {end_page} (Index {start_idx} to {end_idx-1})...")
        
        for i in range(start_idx, end_idx):
            if i < len(reader.pages):
                page = reader.pages[i]
                text += f"\n--- Page {i+1} ---\n"
                text += page.extract_text()
            else:
                print(f"Page {i+1} out of bounds.")
        
        return text

    except Exception as e:
        return f"Error: {e}"

if __name__ == "__main__":
    pdf_path = r"c:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason\controls-engineering-in-frc.pdf"
    # Chapter 15 starts on Page 252. Appendices start on Page 280.
    # Reading 252-280 covers Ch 15, 16, 17.
    print(extract_pages(pdf_path, 252, 280))
