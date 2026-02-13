import pypdf

def extract_toc(pdf_path, output_path):
    try:
        reader = pypdf.PdfReader(pdf_path)
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(f"Number of pages: {len(reader.pages)}\n")
            
            # Try to get the outline
            outline = reader.outline
            if not outline:
                f.write("No outline found in PDF.\n")
                return

            def print_outline(items, level=0):
                for item in items:
                    if isinstance(item, list):
                        print_outline(item, level + 1)
                    else:
                        if hasattr(item, 'title'):
                            # get_destination_page_number might return None if dest is invalid, handle carefully
                            try:
                                page_num = reader.get_destination_page_number(item)
                                if page_num is not None:
                                    page_num += 1
                                else:
                                    page_num = "Unknown"
                            except:
                                page_num = "Unknown"
                                
                            f.write(f"{'  ' * level}- {item.title} (Page {page_num})\n")
                        else:
                            f.write(f"{'  ' * level}- {item} (Unknown Page)\n")

            f.write("Table of Contents:\n")
            print_outline(outline)
            print(f"TOC written to {output_path}")

    except Exception as e:
        print(f"Error reading PDF: {e}")

if __name__ == "__main__":
    pdf_path = r"c:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason\controls-engineering-in-frc.pdf"
    output_path = r"c:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason\toc.txt"
    extract_toc(pdf_path, output_path)
