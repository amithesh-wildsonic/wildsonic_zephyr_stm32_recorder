import argparse
import bson

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Process bson file and print')
    parser.add_argument('bson_file', metavar='bson_file', type=str,
                        help='bson file to process')
    PARSED_ARGS = parser.parse_args()

    doc = bson.loads(open(PARSED_ARGS.bson_file, 'rb').read())

    formatted_doc = {}

    for key in doc.keys():

        if type(doc[key]) == int:
            # Format as 64-bit unsigned hex integer
            formatted_doc[key] = hex(int(doc[key]))
        elif hasattr(doc[key], 'decode'):
            # Leave as is
            formatted_doc[key] = doc[key]
        else:
            # Convert to string
            formatted_doc[key] = str(doc[key])

    print(formatted_doc)