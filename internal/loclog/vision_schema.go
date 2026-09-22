package loclog

import (
	"fmt"

	"github.com/foxglove/mcap/go/mcap"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/reflect/protodesc"
	"google.golang.org/protobuf/reflect/protoreflect"
	"google.golang.org/protobuf/types/descriptorpb"

	"github.com/Rione/ssl-RACOON-Pi3/proto/pb_gen"
)

// visionSchemaID は /in/vision に紐づく protobuf スキーマの ID。
const visionSchemaID = 1

// writeVisionSchema は SSL_WrapperPacket の FileDescriptorSet を MCAP へ書く。
//
// 生の protobuf をスキーマごと埋めておくのが肝 (計画 §6.1)。これがあれば
// Foxglove が単体でデコードでき、後から .proto を探し回らずに済む。
func (s *sink) writeVisionSchema() (uint16, error) {
	md := (&pb_gen.SSL_WrapperPacket{}).ProtoReflect().Descriptor()
	fds, err := fileDescriptorSetFor(md)
	if err != nil {
		return 0, err
	}
	data, err := proto.Marshal(fds)
	if err != nil {
		return 0, fmt.Errorf("loclog: marshal vision descriptor set: %w", err)
	}
	if err := s.mcapW.WriteSchema(&mcap.Schema{
		ID:       visionSchemaID,
		Name:     string(md.FullName()),
		Encoding: "protobuf",
		Data:     data,
	}); err != nil {
		return 0, fmt.Errorf("loclog: write vision schema: %w", err)
	}
	return visionSchemaID, nil
}

// fileDescriptorSetFor はメッセージの定義ファイルと、その import を推移的に集める。
// 依存より先に依存される側が並ぶ順序で返す。
func fileDescriptorSetFor(md protoreflect.MessageDescriptor) (*descriptorpb.FileDescriptorSet, error) {
	set := &descriptorpb.FileDescriptorSet{}
	seen := make(map[string]bool)

	var add func(fd protoreflect.FileDescriptor) error
	add = func(fd protoreflect.FileDescriptor) error {
		if seen[fd.Path()] {
			return nil
		}
		seen[fd.Path()] = true

		imports := fd.Imports()
		for i := 0; i < imports.Len(); i++ {
			if err := add(imports.Get(i).FileDescriptor); err != nil {
				return err
			}
		}
		set.File = append(set.File, protodesc.ToFileDescriptorProto(fd))
		return nil
	}

	if err := add(md.ParentFile()); err != nil {
		return nil, err
	}
	if len(set.File) == 0 {
		return nil, fmt.Errorf("loclog: empty descriptor set for %s", md.FullName())
	}
	return set, nil
}
